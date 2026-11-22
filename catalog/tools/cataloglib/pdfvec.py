"""A small reader for the vector graphs of a PDF (standard library only): the objects, the page content streams and
a content-stream interpreter that returns every stroked or filled path (curves flattened) and every text string with
its position in page coordinates. Written for catalog/tools/extract_harmonicdrive_efficiency.py.

shortcut: unencrypted PDFs with flate streams and object streams only, no images, no font decoding (text is returned
as the bytes of the string, which is readable for the simple fonts of the graphs); upgrade with a PDF library if a
document needs more.
"""
import re
import zlib

TOKEN = re.compile(rb"\((?:\\.|[^\\()])*\)|<[0-9A-Fa-f\s]*>|\[|\]|/[^\s/\[\]()<>]*|[-+]?\d*\.?\d+|[A-Za-z'\"*]+")


def stream(body):
    """(dictionary text, decoded data) of an object body."""
    i = body.find(b"stream")
    if i < 0:
        return body, b""
    head, s = body[:i], i + 6
    if body[s:s + 2] == b"\r\n":
        s += 2
    elif body[s:s + 1] in (b"\n", b"\r"):
        s += 1
    raw = body[s:body.rfind(b"endstream")]
    if b"FlateDecode" in head:
        raw = zlib.decompressobj().decompress(raw)
    return head, raw


def load(path):
    """{object number: body}; later definitions win, objects inside object streams are expanded."""
    data = open(path, "rb").read()
    objs = {}
    for m in re.finditer(rb"(\d+) (\d+) obj\b", data):
        objs[int(m.group(1))] = data[m.end():data.find(b"endobj", m.end())]
    for body in list(objs.values()):
        if b"/ObjStm" in body[:400]:
            head, d = stream(body)
            n, first = (int(re.search(rb"/%s (\d+)" % k, head).group(1)) for k in (b"N", b"First"))
            nums = list(map(int, d[:first].split()))
            for i in range(n):
                end = nums[2 * i + 3] if i + 1 < n else len(d) - first
                objs.setdefault(nums[2 * i], d[first + nums[2 * i + 1]:first + end])
    return data, objs


def pages(data, objs):
    """Object numbers of the pages in page order."""
    out = []

    def walk(n):
        kids = re.search(rb"/Kids\s*\[([^\]]*)\]", objs[n])
        if kids and not re.search(rb"/Type\s*/Page\b", objs[n]):
            for m in re.finditer(rb"(\d+) 0 R", kids.group(1)):
                walk(int(m.group(1)))
        else:
            out.append(n)

    root = int(re.findall(rb"/Root (\d+) 0 R", data)[-1])
    walk(int(re.search(rb"/Pages (\d+) 0 R", objs[root]).group(1)))
    return out


def content(objs, page):
    m = re.search(rb"/Contents\s*(\[[^\]]*\]|\d+ 0 R)", objs[page])
    return b"\n".join(stream(objs[int(p)])[1] for p in re.findall(rb"(\d+) 0 R", m.group(1)))


def _mul(a, b):
    return (a[0] * b[0] + a[1] * b[2], a[0] * b[1] + a[1] * b[3], a[2] * b[0] + a[3] * b[2], a[2] * b[1] + a[3] * b[3],
            a[4] * b[0] + a[5] * b[2] + b[4], a[4] * b[1] + a[5] * b[3] + b[5])


def _pt(m, x, y):
    return (x * m[0] + y * m[2] + m[4], x * m[1] + y * m[3] + m[5])


def _string(token):
    s, out, i = token[1:-1], bytearray(), 0
    while i < len(s):
        c = s[i]
        if c == 92:
            i += 1
            c = s[i]
            if c in b"nrtbf":
                out.append({110: 10, 114: 13, 116: 9, 98: 8, 102: 12}[c])
            elif 48 <= c <= 55:
                j = i
                while j < i + 3 and j < len(s) and 48 <= s[j] <= 55:
                    j += 1
                out.append(int(s[i:j], 8) & 255)
                i = j - 1
            else:
                out.append(c)
        else:
            out.append(c)
        i += 1
    return bytes(out)


def interpret(data):
    """(paths, texts): a path is {'kind': 'S' (stroke) or 'f' (fill), 'subs': [[(x, y), ...]], 'rgb', 'w' (line width
    in page units)}; a text is (string, x, y, size)."""
    saved, ctm, stroke, fill, lw = [], (1, 0, 0, 1, 0, 0), (0, 0, 0), (0, 0, 0), 1.0
    cur, sub, paths, texts, stack = [], None, [], [], []
    tm = tlm = (1, 0, 0, 1, 0, 0)
    size = 1.0

    def cmyk(a):
        c, m, y, k = (float(v) for v in a[:4])
        return ((1 - c) * (1 - k), (1 - m) * (1 - k), (1 - y) * (1 - k))

    for m in TOKEN.finditer(data):
        t = m.group(0)
        if t[:1] in b"([/<" or t[:1].isdigit() or t[:1] in b"-+.":
            stack.append(t)
            continue
        if t == b"]":
            i = len(stack) - 1
            while i >= 0 and stack[i] != b"[":
                i -= 1
            if i >= 0:
                arr = stack[i + 1:]
                del stack[i:]
                stack.append(arr)
            continue
        op, a, stack = t, stack, []
        try:
            f = [float(v) for v in a if isinstance(v, bytes) and re.fullmatch(rb"[-+]?\d*\.?\d+", v)]
            if op == b"q":
                saved.append((ctm, stroke, fill, lw))
            elif op == b"Q":
                ctm, stroke, fill, lw = saved.pop()
            elif op == b"cm":
                ctm = _mul(tuple(f[:6]), ctm)
            elif op == b"w":
                lw = f[0]
            elif op == b"RG":
                stroke = tuple(f[:3])
            elif op == b"rg":
                fill = tuple(f[:3])
            elif op == b"G":
                stroke = (f[0],) * 3
            elif op == b"g":
                fill = (f[0],) * 3
            elif op == b"K":
                stroke = cmyk(f)
            elif op == b"k":
                fill = cmyk(f)
            elif op == b"m":
                sub = [_pt(ctm, f[0], f[1])]
                cur.append(sub)
            elif op == b"l":
                sub.append(_pt(ctm, f[0], f[1]))
            elif op == b"c":
                p0, p1, p2, p3 = sub[-1], _pt(ctm, f[0], f[1]), _pt(ctm, f[2], f[3]), _pt(ctm, f[4], f[5])
                for k in range(1, 17):
                    u = k / 16
                    v = 1 - u
                    sub.append(tuple(v ** 3 * p0[j] + 3 * v * v * u * p1[j] + 3 * v * u * u * p2[j] + u ** 3 * p3[j]
                                     for j in (0, 1)))
            elif op in (b"v", b"y"):
                sub.append(_pt(ctm, f[2], f[3]))
            elif op == b"re":
                x, y, w, h = f[:4]
                cur.append([_pt(ctm, x, y), _pt(ctm, x + w, y), _pt(ctm, x + w, y + h), _pt(ctm, x, y + h),
                            _pt(ctm, x, y)])
            elif op == b"h":
                if sub:
                    sub.append(sub[0])
            elif op in (b"S", b"s", b"B", b"b", b"B*", b"b*", b"f", b"F", b"f*"):
                kind = "f" if op in (b"f", b"F", b"f*") else "S"
                if cur:
                    paths.append({"kind": kind, "subs": cur, "rgb": fill if kind == "f" else stroke,
                                  "w": lw * abs(ctm[0] or ctm[1])})
                cur = []
            elif op == b"n":
                cur = []
            elif op == b"BT":
                tm = tlm = (1, 0, 0, 1, 0, 0)
            elif op == b"Tf":
                size = f[0]
            elif op == b"Tm":
                tm = tlm = tuple(f[:6])
            elif op in (b"Td", b"TD"):
                tlm = tm = _mul((1, 0, 0, 1, f[0], f[1]), tlm)
            elif op in (b"Tj", b"TJ", b"'"):
                s = b""
                for x in (a[0] if isinstance(a[0], list) else [a[0]]):
                    if x[:1] == b"(":
                        s += _string(x)
                    elif x[:1] == b"<":
                        s += bytes.fromhex(x[1:-1].decode().replace(" ", ""))
                p = _mul(tm, ctm)
                texts.append((s.decode("latin1"), p[4], p[5], size * abs(p[3])))
        except (IndexError, ValueError):
            continue  # an operator with other operands than expected (marked content, clipping): not a graph element
    return paths, texts

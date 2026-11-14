"""mdBook preprocessor (docs/book.toml): links from docs/ to files elsewhere in the repository, such as
`../crates/machine-delta-d3/default.toml`, point to the file on GitHub, since the site holds only docs/.

mdBook calls it as `mdbook_repo_links.py supports <renderer>` (answer: yes) and then with `[context, book]` as JSON
on stdin, expecting the book back on stdout.
"""

import json
import posixpath
import re
import sys

BLOB = "https://github.com/Sparkz400/KinematicsSimulation/blob/main/"
# A Markdown inline link's target: `](target)` or `](target#anchor)`, not a web or mail link.
LINK = re.compile(r"\]\((?![a-z][a-z0-9+.-]*:)([^)#\s]+)")


def rewrite(content, chapter_path):
    base = posixpath.dirname(chapter_path.replace("\\", "/"))

    def repl(m):
        target = posixpath.normpath(posixpath.join("docs", base, m.group(1)))
        if target.startswith("docs/"):
            return m.group(0)
        return "](" + BLOB + target

    return LINK.sub(repl, content)


def walk(items):
    for item in items:
        chapter = item.get("Chapter") if isinstance(item, dict) else None
        if chapter:
            if chapter.get("path"):
                chapter["content"] = rewrite(chapter["content"], chapter["path"])
            walk(chapter.get("sub_items", []))


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "supports":
        sys.exit(0)
    if len(sys.argv) > 1 and sys.argv[1] == "test":
        assert rewrite("[a](../crates/x.toml#k)", "x.md") == "[a](" + BLOB + "crates/x.toml#k)"
        assert rewrite("[a](../../README.md)", "de/x.md") == "[a](" + BLOB + "README.md)"
        assert rewrite("[a](../fmi.md#b) [c](https://x.org/a)", "de/x.md") == "[a](../fmi.md#b) [c](https://x.org/a)"
        return
    # UTF-8 both ways: the default encoding on Windows would mangle the German pages.
    _context, book = json.load(sys.stdin.buffer)
    walk(book.get("items", book.get("sections", [])))
    sys.stdout.buffer.write(json.dumps(book).encode("utf-8"))


if __name__ == "__main__":
    main()

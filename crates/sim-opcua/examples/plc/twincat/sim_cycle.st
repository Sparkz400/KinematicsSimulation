(*
    A PLC cycle against the KinematicsSimulation link (sim-opcua), for TwinCAT 3 with TF6100 (library
    Tc3_PLCopen_OpcUa). Uses SIM_Access, SIM_MC_Power and SIM_MC_MoveAbsolute from sim_plcopen.st in this folder.

    NOT COMPILED: written against the Tc3_PLCopen_OpcUa documentation, not built in TwinCAT (none was available).
    Names that can differ between library versions are marked "CHECK".

    PRG_SimCycle:
      - connects with Basic256Sha256 SignAndEncrypt (or None, see bSecure), optionally with user name and password;
      - finds the namespace index from the URI 'urn:kinematics-simulation';
      - gets the node handles of the position and following error of one axis once and reads both every cycle,
        with the handles kept (one round trip per read; a read is started again as soon as the last one is done);
      - powers the axis (MC_Power), moves it with MC_MoveAbsolute, then writes a path and runs it
        (command.path, command.run_path) and waits until status.busy falls;
      - keeps the largest following error seen, switches off and disconnects.
    Run it in a task of 10 ms; sim-opcua then reports the task's write and read intervals (status.client_*).

    Data types as sim-opcua serves them (docs/plc.md, "Node ids and data types"): Double -> LREAL, Boolean -> BOOL,
    UInt16 -> UINT, Int32 -> DINT, UInt64 -> ULINT, String -> STRING. A PLC variable of another type (REAL for a
    Double) makes UA_Read fail with a type mismatch.
*)

FUNCTION_BLOCK SIM_CyclicRead
(* Reads one LREAL variable over and over through a node handle that is got once. *)
VAR_INPUT
    bEnable     : BOOL;
    hConnection : DWORD;
    nNamespace  : UINT;
    sNode       : STRING(255);
END_VAR
VAR_OUTPUT
    fValue  : LREAL;
    nReads  : UDINT;     (* reads done *)
    bError  : BOOL;
    nErrorID : DWORD;
END_VAR
VAR
    fbGetHandle : UA_NodeGetHandle;
    fbRead      : UA_Read;
    fbRelease   : UA_NodeReleaseHandle;
    stNodeID    : ST_UANodeID;
    stInfo      : ST_UANodeAdditionalInfo;   (* CHECK: member name of the attribute id *)
    hNode       : DWORD;
    nStep       : INT;
END_VAR

CASE nStep OF
0:  IF bEnable THEN
        stNodeID.nNamespaceIndex := nNamespace;
        stNodeID.sIdentifier := sNode;
        stNodeID.eIdentifierType := eUAIdentifierType.String;
        stInfo.nAttributeID := 13;   (* the Value attribute *)
        bError := FALSE; nErrorID := 0;
        nStep := 1;
    END_IF
1:  fbGetHandle(Execute := TRUE, ConnectionHdl := hConnection, NodeID := stNodeID);
    IF fbGetHandle.Done THEN
        hNode := fbGetHandle.NodeHdl; fbGetHandle(Execute := FALSE); nStep := 2;
    ELSIF fbGetHandle.Error THEN
        nErrorID := fbGetHandle.ErrorID; fbGetHandle(Execute := FALSE); bError := TRUE; nStep := 9;
    END_IF
2:  (* one read; once it is done the next starts in the same cycle, so a read that returns within a cycle runs
       every cycle *)
    fbRead(Execute := TRUE, ConnectionHdl := hConnection, NodeHdl := hNode, stNodeAddInfo := stInfo,
           cbData := SIZEOF(fValue), pVariable := ADR(fValue));
    IF fbRead.Done OR fbRead.Error THEN
        IF fbRead.Error THEN bError := TRUE; nErrorID := fbRead.ErrorID; ELSE nReads := nReads + 1; END_IF
        fbRead(Execute := FALSE);
        IF bEnable THEN
            fbRead(Execute := TRUE, ConnectionHdl := hConnection, NodeHdl := hNode, stNodeAddInfo := stInfo,
                   cbData := SIZEOF(fValue), pVariable := ADR(fValue));
        ELSE
            nStep := 3;
        END_IF
    END_IF
3:  fbRelease(Execute := TRUE, ConnectionHdl := hConnection, NodeHdl := hNode);
    IF fbRelease.Done OR fbRelease.Error THEN fbRelease(Execute := FALSE); nStep := 0; END_IF
9:  IF NOT bEnable THEN nStep := 0; END_IF
END_CASE
END_FUNCTION_BLOCK

PROGRAM PRG_SimCycle
VAR
    sServerUrl   : STRING(255) := 'opc.tcp://127.0.0.1:4840';   (* sim-opcua on the same PC *)
    bSecure      : BOOL := TRUE;                                 (* FALSE: SecurityPolicy None *)
    sUser        : STRING(63) := '';                             (* empty: anonymous; else as sim-opcua --user *)
    sPassword    : STRING(63) := '';
    sAxis        : STRING(31) := 'x';                            (* an axis id of the simulated machine *)
    fTarget      : LREAL := 0.4;                                 (* m, inside the axis range *)
    (* The path in the text of a path file, one piece per line ($N), one position per axis (here x and z of the
       T-gantry, which starts at x 0.1, z 0.35). *)
    sPath        : STRING(255) := 'line,0.4,0.25$Nline,0.1,0.25$Nline,0.1,0.35';
    (* sim-opcua counts every Read request: two reads a cycle show as intervals of half a cycle. FALSE reads the
       position alone, one request a cycle, so status.client_read_* give the task's cycle and jitter. *)
    bReadFollowingError : BOOL := TRUE;

    fbConnect    : UA_Connect;
    fbNamespace  : UA_GetNamespaceIndex;
    fbDisconnect : UA_Disconnect;
    stSession    : ST_UASessionConnectInfo;
    hConnection  : DWORD;
    nNamespace   : UINT;
    fbPower      : SIM_MC_Power;
    fbMove       : SIM_MC_MoveAbsolute;
    fbWrite      : SIM_Access;
    fbBusy       : SIM_Access;
    fbPosition   : SIM_CyclicRead;
    fbError      : SIM_CyclicRead;
    bTrue        : BOOL := TRUE;
    bBusy        : BOOL;
    bBusySeen    : BOOL;
    fLargestError : LREAL;   (* m, the largest following error read *)
    nStep        : INT;
END_VAR

CASE nStep OF
0:  stSession.sApplicationName := 'TwinCAT sim client';
    stSession.eTransportProfileUri := eUATransportProfileUri.UATcp;
    stSession.nSessionTimeout := T#60S;
    IF bSecure THEN
        (* TF6100 creates its own client certificate on the first secured connection (CHECK: its PKI folder in the
           TwinCAT documentation). sim-opcua refuses it once and stores it in pki/rejected; move it to pki/trusted,
           or start sim-opcua with --trust-all while commissioning. The server's certificate must be trusted in
           TF6100's PKI as well. *)
        stSession.eSecurityMode := eUASecurityMsgMode.SignAndEncrypt;
        stSession.eSecurityPolicyUri := eUASecurityPolicyUri.Basic256Sha256;
    ELSE
        stSession.eSecurityMode := eUASecurityMsgMode.None;
        stSession.eSecurityPolicyUri := eUASecurityPolicyUri.None;
    END_IF
    IF LEN(sUser) > 0 THEN
        (* CHECK: member names of the user identity token *)
        stSession.stUserIdentityToken.eUserIdentityTokenType := eUAUserIdentityTokenType.UserName;
        stSession.stUserIdentityToken.sUserName := sUser;
        stSession.stUserIdentityToken.sPassword := sPassword;
    END_IF
    fbConnect(Execute := TRUE, ServerURL := sServerUrl, SessionConnectInfo := stSession, Timeout := T#5S);
    IF fbConnect.Done THEN hConnection := fbConnect.ConnectionHdl; fbConnect(Execute := FALSE); nStep := 10;
    ELSIF fbConnect.Error THEN fbConnect(Execute := FALSE); nStep := 900; END_IF
10: fbNamespace(Execute := TRUE, ConnectionHdl := hConnection, NamespaceUri := 'urn:kinematics-simulation', Timeout := T#5S);
    IF fbNamespace.Done THEN nNamespace := fbNamespace.NamespaceIndex; fbNamespace(Execute := FALSE); nStep := 20;
    ELSIF fbNamespace.Error THEN fbNamespace(Execute := FALSE); nStep := 900; END_IF
20: (* enable the drives *)
    fbPower.Enable := TRUE;
    IF fbPower.Status THEN nStep := 30; END_IF
30: (* MC_MoveAbsolute *)
    fbMove.Position := fTarget; fbMove.Velocity := 0.5; fbMove.Acceleration := 5.0; fbMove.Deceleration := 5.0;
    fbMove.Jerk := 100.0;
    fbMove.Execute := TRUE;
    IF fbMove.Done THEN fbMove.Execute := FALSE; nStep := 40;
    ELSIF fbMove.Error OR fbMove.CommandAborted THEN nStep := 900; END_IF
40: (* the path: its text, then command.run_path *)
    fbWrite(bExecute := FALSE);
    fbWrite(bExecute := TRUE, hConnection := hConnection, nNamespace := nNamespace, sNode := 'command.path',
            bWrite := TRUE, pData := ADR(sPath), cbData := LEN(sPath));   (* CHECK: cbData of a STRING *)
    nStep := 41;
41: fbWrite(bExecute := TRUE);
    IF fbWrite.bDone THEN nStep := 42; ELSIF fbWrite.bError THEN nStep := 900; END_IF
42: fbWrite(bExecute := FALSE);
    fbWrite(bExecute := TRUE, hConnection := hConnection, nNamespace := nNamespace, sNode := 'command.run_path',
            bWrite := TRUE, pData := ADR(bTrue), cbData := SIZEOF(bTrue));
    nStep := 43;
43: fbWrite(bExecute := TRUE);
    IF fbWrite.bDone THEN bBusySeen := FALSE; nStep := 50; ELSIF fbWrite.bError THEN nStep := 900; END_IF
50: (* poll status.busy until the path is over: it rises within a loop cycle or two and falls at the end *)
    fbBusy(bExecute := FALSE);
    fbBusy(bExecute := TRUE, hConnection := hConnection, nNamespace := nNamespace, sNode := 'status.busy',
           bWrite := FALSE, pData := ADR(bBusy), cbData := SIZEOF(bBusy));
    nStep := 51;
51: fbBusy(bExecute := TRUE);
    IF fbBusy.bDone THEN
        IF bBusy THEN bBusySeen := TRUE; nStep := 50;
        ELSIF bBusySeen THEN nStep := 60;
        ELSE nStep := 50; END_IF
    ELSIF fbBusy.bError THEN nStep := 900; END_IF
60: (* switch off and disconnect *)
    fbPower.Enable := FALSE;
    IF NOT fbPower.Status THEN nStep := 70; END_IF
70: fbDisconnect(Execute := TRUE, ConnectionHdl := hConnection, Timeout := T#5S);
    IF fbDisconnect.Done OR fbDisconnect.Error THEN fbDisconnect(Execute := FALSE); nStep := 100; END_IF
100: ; (* finished: fLargestError holds the largest following error *)
900: ; (* error: look at the blocks' ErrorID and at status.refused on the server *)
END_CASE

(* every cycle from the namespace on: position and following error of the axis *)
fbPosition(bEnable := nStep >= 20 AND nStep < 70, hConnection := hConnection, nNamespace := nNamespace,
           sNode := CONCAT(CONCAT('signal.axis.', sAxis), '.position'));
fbError(bEnable := bReadFollowingError AND nStep >= 20 AND nStep < 70, hConnection := hConnection, nNamespace := nNamespace,
        sNode := CONCAT(CONCAT('signal.axis.', sAxis), '.following_error'));
IF ABS(fbError.fValue) > fLargestError THEN fLargestError := ABS(fbError.fValue); END_IF

IF nStep >= 20 AND nStep < 70 THEN
    fbPower(hConnection := hConnection, nNamespace := nNamespace, sAxis := sAxis);
    fbMove(hConnection := hConnection, nNamespace := nNamespace, sAxis := sAxis);
END_IF
END_PROGRAM

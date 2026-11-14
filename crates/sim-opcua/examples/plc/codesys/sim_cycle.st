(*
    A PLC cycle against the KinematicsSimulation link (sim-opcua), for CODESYS V3.5 with the CODESYS OPC UA Client
    SL (PLCopen "OPC UA Client for IEC 61131-3" blocks). Uses SIM_Access, SIM_MC_Power and SIM_MC_MoveAbsolute from
    sim_plcopen.st in this folder.

    NOT COMPILED: follows the PLCopen OPC UA client specification (v1.1), not built in CODESYS (none was available).
    Type and member names can differ by library version (marked "CHECK"); depending on the library, the names may
    need the library's namespace prefix.

    PRG_SimCycle does what ../twincat/sim_cycle.st does: connect with Basic256Sha256 SignAndEncrypt (or None) and
    optionally user name and password, find the namespace index, power the axis, MC_MoveAbsolute, write a path and
    run it, wait until status.busy falls, switch off and disconnect; meanwhile it reads the axis position and
    following error every cycle through node handles got once. Run it in a task of 10 ms.

    Data types (docs/plc.md, "Node ids and data types"): Double -> LREAL, Boolean -> BOOL, UInt16 -> UINT,
    Int32 -> DINT, UInt64 -> ULINT, String -> STRING. The value of UA_Read/UA_Write is an IN_OUT of type ANY: its
    IEC type is what goes on the wire, so it has to be the one in the table.
*)

FUNCTION_BLOCK SIM_CyclicRead
(* Reads one LREAL variable over and over through a node handle that is got once. *)
VAR_INPUT
    xEnable     : BOOL;
    hConnection : DWORD;
    uiNamespace : UINT;
    sNode       : STRING(255);
END_VAR
VAR_OUTPUT
    lrValue   : LREAL;
    udiReads  : UDINT;
    xError    : BOOL;
    dwErrorID : DWORD;
END_VAR
VAR
    fbGetHandle : UA_NodeGetHandle;
    fbRead      : UA_Read;
    fbRelease   : UA_NodeReleaseHandle;
    stNodeID    : UANodeID;                (* CHECK: type name *)
    stInfo      : UANodeAdditionalInfo;    (* CHECK: type name *)
    hNode       : DWORD;
    iStep       : INT;
END_VAR

CASE iStep OF
0:  IF xEnable THEN
        stNodeID.NamespaceIndex := uiNamespace;
        stNodeID.Identifier := sNode;
        stNodeID.IdentifierType := UAIdentifierType.UAIdentifierType_String;   (* CHECK: enumeration name *)
        stInfo.AttributeID := 13;   (* the Value attribute *)
        xError := FALSE; dwErrorID := 0;
        iStep := 1;
    END_IF
1:  fbGetHandle(Execute := TRUE, ConnectionHdl := hConnection, NodeID := stNodeID, Timeout := T#5S);
    IF fbGetHandle.Done THEN
        hNode := fbGetHandle.NodeHdl; fbGetHandle(Execute := FALSE, ConnectionHdl := hConnection, NodeID := stNodeID);
        iStep := 2;
    ELSIF fbGetHandle.Error THEN
        dwErrorID := fbGetHandle.ErrorID; fbGetHandle(Execute := FALSE, ConnectionHdl := hConnection, NodeID := stNodeID);
        xError := TRUE; iStep := 9;
    END_IF
2:  (* once a read is done the next starts in the same cycle *)
    fbRead(Execute := TRUE, ConnectionHdl := hConnection, NodeHdl := hNode, NodeAddInfo := stInfo, Timeout := T#5S,
           Variable := lrValue);
    IF fbRead.Done OR fbRead.Error THEN
        IF fbRead.Error THEN xError := TRUE; dwErrorID := fbRead.ErrorID; ELSE udiReads := udiReads + 1; END_IF
        fbRead(Execute := FALSE, Variable := lrValue);
        IF xEnable THEN
            fbRead(Execute := TRUE, ConnectionHdl := hConnection, NodeHdl := hNode, NodeAddInfo := stInfo,
                   Timeout := T#5S, Variable := lrValue);
        ELSE
            iStep := 3;
        END_IF
    END_IF
3:  fbRelease(Execute := TRUE, ConnectionHdl := hConnection, NodeHdl := hNode, Timeout := T#5S);
    IF fbRelease.Done OR fbRelease.Error THEN
        fbRelease(Execute := FALSE, ConnectionHdl := hConnection, NodeHdl := hNode); iStep := 0;
    END_IF
9:  IF NOT xEnable THEN iStep := 0; END_IF
END_CASE
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_WriteString
(* One write of a STRING variable: get the handle, write, release. *)
VAR_INPUT
    xExecute    : BOOL;
    hConnection : DWORD;
    uiNamespace : UINT;
    sNode       : STRING(255);
END_VAR
VAR_IN_OUT
    sValue : STRING(255);
END_VAR
VAR_OUTPUT
    xDone : BOOL; xError : BOOL; dwErrorID : DWORD;
END_VAR
VAR
    fbGetHandle : UA_NodeGetHandle;
    fbWrite     : UA_Write;
    fbRelease   : UA_NodeReleaseHandle;
    stNodeID    : UANodeID;                (* CHECK: type name *)
    stInfo      : UANodeAdditionalInfo;    (* CHECK: type name *)
    hNode       : DWORD;
    iStep       : INT;
    xLast       : BOOL;
END_VAR

IF xExecute AND NOT xLast THEN
    xDone := FALSE; xError := FALSE; dwErrorID := 0;
    stNodeID.NamespaceIndex := uiNamespace;
    stNodeID.Identifier := sNode;
    stNodeID.IdentifierType := UAIdentifierType.UAIdentifierType_String;   (* CHECK *)
    stInfo.AttributeID := 13;
    iStep := 1;
END_IF
xLast := xExecute;

CASE iStep OF
1:  fbGetHandle(Execute := TRUE, ConnectionHdl := hConnection, NodeID := stNodeID, Timeout := T#5S);
    IF fbGetHandle.Done THEN
        hNode := fbGetHandle.NodeHdl; fbGetHandle(Execute := FALSE, ConnectionHdl := hConnection, NodeID := stNodeID);
        iStep := 2;
    ELSIF fbGetHandle.Error THEN
        dwErrorID := fbGetHandle.ErrorID; fbGetHandle(Execute := FALSE, ConnectionHdl := hConnection, NodeID := stNodeID);
        xError := TRUE; iStep := 0;
    END_IF
2:  fbWrite(Execute := TRUE, ConnectionHdl := hConnection, NodeHdl := hNode, NodeAddInfo := stInfo, Timeout := T#5S,
            Variable := sValue);
    IF fbWrite.Done OR fbWrite.Error THEN
        xError := fbWrite.Error; dwErrorID := fbWrite.ErrorID;
        fbWrite(Execute := FALSE, Variable := sValue);
        iStep := 3;
    END_IF
3:  fbRelease(Execute := TRUE, ConnectionHdl := hConnection, NodeHdl := hNode, Timeout := T#5S);
    IF fbRelease.Done OR fbRelease.Error THEN
        fbRelease(Execute := FALSE, ConnectionHdl := hConnection, NodeHdl := hNode);
        xDone := NOT xError; iStep := 0;
    END_IF
END_CASE
END_FUNCTION_BLOCK

PROGRAM PRG_SimCycle
VAR
    sServerUrl   : STRING(255) := 'opc.tcp://127.0.0.1:4840';   (* sim-opcua on the same PC *)
    xSecure      : BOOL := TRUE;                                 (* FALSE: SecurityPolicy None *)
    sUser        : STRING(63) := '';                             (* empty: anonymous; else as sim-opcua --user *)
    sPassword    : STRING(63) := '';
    sAxis        : STRING(31) := 'x';
    lrTarget     : LREAL := 0.4;
    sPath        : STRING(255) := 'line,0.4,0.25$Nline,0.1,0.25$Nline,0.1,0.35';
    (* FALSE: the position alone, one Read request a cycle, so that status.client_read_* show the task's cycle *)
    xReadFollowingError : BOOL := TRUE;

    fbConnect    : UA_Connect;
    fbNamespace  : UA_NamespaceGetIndex;
    fbDisconnect : UA_Disconnect;
    stSession    : UASessionConnectInfo;   (* CHECK: type and member names *)
    hConnection  : DWORD;
    uiNamespace  : UINT;
    fbPower      : SIM_MC_Power;
    fbMove       : SIM_MC_MoveAbsolute;
    fbPath       : SIM_WriteString;
    fbAccess     : SIM_Access;
    fbPosition   : SIM_CyclicRead;
    fbError      : SIM_CyclicRead;
    xFlag        : BOOL;
    lrUnused     : LREAL;
    uiUnused     : UINT;
    xBusySeen    : BOOL;
    lrLargestError : LREAL;
    iStep        : INT;
END_VAR

CASE iStep OF
0:  stSession.ApplicationName := 'CODESYS sim client';
    IF xSecure THEN
        (* The client certificate: create it in the CODESYS security screen (Devices > Security; CHECK: name in your
           version) for the OPC UA client. sim-opcua refuses it once and stores it in pki/rejected; move it to
           pki/trusted, or run sim-opcua with --trust-all while commissioning. Trust the server's certificate
           (pki/own/cert.der) in the CODESYS certificate store. *)
        stSession.SecurityMsgMode := UASecurityMsgMode.UASecurityMsgMode_SignAndEncrypt;                (* CHECK *)
        stSession.SecurityPolicyUri := 'http://opcfoundation.org/UA/SecurityPolicy#Basic256Sha256';      (* CHECK *)
    ELSE
        stSession.SecurityMsgMode := UASecurityMsgMode.UASecurityMsgMode_None;                          (* CHECK *)
        stSession.SecurityPolicyUri := 'http://opcfoundation.org/UA/SecurityPolicy#None';                (* CHECK *)
    END_IF
    IF LEN(sUser) > 0 THEN
        stSession.UserIdentityToken.UserIdentityTokenType := UAUserIdentityTokenType.UAUITT_Username;   (* CHECK *)
        stSession.UserIdentityToken.TokenParam1 := sUser;                                               (* CHECK *)
        stSession.UserIdentityToken.TokenParam2 := sPassword;                                           (* CHECK *)
    END_IF
    fbConnect(Execute := TRUE, ServerEndpointUrl := sServerUrl, SessionConnectInfo := stSession, Timeout := T#5S);
    IF fbConnect.Done THEN hConnection := fbConnect.ConnectionHdl; fbConnect(Execute := FALSE); iStep := 10;
    ELSIF fbConnect.Error THEN fbConnect(Execute := FALSE); iStep := 900; END_IF
10: fbNamespace(Execute := TRUE, ConnectionHdl := hConnection, NamespaceUri := 'urn:kinematics-simulation', Timeout := T#5S);
    IF fbNamespace.Done THEN uiNamespace := fbNamespace.NamespaceIndex; fbNamespace(Execute := FALSE); iStep := 20;
    ELSIF fbNamespace.Error THEN fbNamespace(Execute := FALSE); iStep := 900; END_IF
20: fbPower.Enable := TRUE;
    IF fbPower.Status THEN iStep := 30; END_IF
30: fbMove.Position := lrTarget; fbMove.Velocity := 0.5; fbMove.Acceleration := 5.0; fbMove.Deceleration := 5.0;
    fbMove.Jerk := 100.0;
    fbMove.Execute := TRUE;
    IF fbMove.Done THEN fbMove.Execute := FALSE; iStep := 40;
    ELSIF fbMove.Error OR fbMove.CommandAborted THEN iStep := 900; END_IF
40: fbPath(xExecute := FALSE, sValue := sPath);
    fbPath(xExecute := TRUE, hConnection := hConnection, uiNamespace := uiNamespace, sNode := 'command.path', sValue := sPath);
    IF fbPath.xDone THEN iStep := 42; ELSIF fbPath.xError THEN iStep := 900; ELSE iStep := 41; END_IF
41: fbPath(xExecute := TRUE, sValue := sPath);
    IF fbPath.xDone THEN iStep := 42; ELSIF fbPath.xError THEN iStep := 900; END_IF
42: xFlag := TRUE;
    fbAccess(xExecute := FALSE, xValue := xFlag, lrValue := lrUnused, uiValue := uiUnused);
    fbAccess(xExecute := TRUE, hConnection := hConnection, uiNamespace := uiNamespace, sNode := 'command.run_path',
             eKind := SIM_BOOL, xWrite := TRUE, xValue := xFlag, lrValue := lrUnused, uiValue := uiUnused);
    iStep := 43;
43: fbAccess(xExecute := TRUE, xValue := xFlag, lrValue := lrUnused, uiValue := uiUnused);
    IF fbAccess.xDone THEN xBusySeen := FALSE; iStep := 50; ELSIF fbAccess.xError THEN iStep := 900; END_IF
50: (* poll status.busy until the path is over *)
    fbAccess(xExecute := FALSE, xValue := xFlag, lrValue := lrUnused, uiValue := uiUnused);
    fbAccess(xExecute := TRUE, hConnection := hConnection, uiNamespace := uiNamespace, sNode := 'status.busy',
             eKind := SIM_BOOL, xWrite := FALSE, xValue := xFlag, lrValue := lrUnused, uiValue := uiUnused);
    iStep := 51;
51: fbAccess(xExecute := TRUE, xValue := xFlag, lrValue := lrUnused, uiValue := uiUnused);
    IF fbAccess.xDone THEN
        IF xFlag THEN xBusySeen := TRUE; iStep := 50;
        ELSIF xBusySeen THEN iStep := 60;
        ELSE iStep := 50; END_IF
    ELSIF fbAccess.xError THEN iStep := 900; END_IF
60: fbPower.Enable := FALSE;
    IF NOT fbPower.Status THEN iStep := 70; END_IF
70: fbDisconnect(Execute := TRUE, ConnectionHdl := hConnection, Timeout := T#5S);
    IF fbDisconnect.Done OR fbDisconnect.Error THEN fbDisconnect(Execute := FALSE); iStep := 100; END_IF
100: ; (* finished: lrLargestError holds the largest following error *)
900: ; (* error: look at the blocks' ErrorID and at status.refused on the server *)
END_CASE

fbPosition(xEnable := iStep >= 20 AND iStep < 70, hConnection := hConnection, uiNamespace := uiNamespace,
           sNode := CONCAT(CONCAT('signal.axis.', sAxis), '.position'));
fbError(xEnable := xReadFollowingError AND iStep >= 20 AND iStep < 70, hConnection := hConnection, uiNamespace := uiNamespace,
        sNode := CONCAT(CONCAT('signal.axis.', sAxis), '.following_error'));
IF ABS(fbError.lrValue) > lrLargestError THEN lrLargestError := ABS(fbError.lrValue); END_IF

IF iStep >= 20 AND iStep < 70 THEN
    fbPower(hConnection := hConnection, uiNamespace := uiNamespace, sAxis := sAxis);
    fbMove(hConnection := hConnection, uiNamespace := uiNamespace, sAxis := sAxis);
END_IF
END_PROGRAM

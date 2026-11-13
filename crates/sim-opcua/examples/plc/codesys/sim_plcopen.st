(*
    PLCopen motion over OPC UA to the KinematicsSimulation link (sim-opcua), for CODESYS V3.5.
    Library: CODESYS OPC UA Client SL (the PLCopen "OPC UA Client for IEC 61131-3" function blocks UA_Connect,
    UA_NamespaceGetIndex, UA_NodeGetHandle, UA_Read, UA_Write, UA_NodeReleaseHandle, UA_Disconnect).

    NOT COMPILED: this file follows the PLCopen OPC UA client specification (v1.1) and has not been built in
    CODESYS. Type and member names can differ by library version (marked "CHECK"); depending on the library, the
    names may need the library's namespace prefix.

    The structure is the one of ../twincat/sim_plcopen.st. The PLCopen UA_Read and UA_Write take the value as
    VAR_IN_OUT of type ANY, so SIM_Access keeps one buffer per data type instead of a pointer.
*)

TYPE E_SimKind : (SIM_BOOL, SIM_LREAL, SIM_UINT); END_TYPE

FUNCTION_BLOCK SIM_Access
VAR_INPUT
    xExecute    : BOOL;
    hConnection : DWORD;
    uiNamespace : UINT;
    sNode       : STRING(255);
    eKind       : E_SimKind;
    xWrite      : BOOL;     (* TRUE: write the value below; FALSE: read into it *)
END_VAR
VAR_IN_OUT
    xValue  : BOOL;
    lrValue : LREAL;
    uiValue : UINT;
END_VAR
VAR_OUTPUT
    xDone    : BOOL;
    xBusy    : BOOL;
    xError   : BOOL;
    dwErrorID : DWORD;
END_VAR
VAR
    fbGetHandle : UA_NodeGetHandle;
    fbReadBool, fbReadReal, fbReadUint    : UA_Read;
    fbWriteBool, fbWriteReal, fbWriteUint : UA_Write;
    fbRelease   : UA_NodeReleaseHandle;
    stNodeID    : UANodeID;                (* CHECK: type name *)
    stInfo      : UANodeAdditionalInfo;    (* CHECK: type name *)
    hNode       : DWORD;
    iStep       : INT;
    xLast       : BOOL;
    xAccessDone, xAccessError : BOOL;
    dwAccessError : DWORD;
END_VAR

IF xExecute AND NOT xLast THEN
    xDone := FALSE; xError := FALSE; dwErrorID := 0; xBusy := TRUE;
    stNodeID.NamespaceIndex := uiNamespace;
    stNodeID.Identifier := sNode;
    stNodeID.IdentifierType := UAIdentifierType.UAIdentifierType_String;   (* CHECK: enumeration name *)
    stInfo.AttributeID := 13;   (* the Value attribute *)
    iStep := 1;
END_IF
xLast := xExecute;

CASE iStep OF
1:  fbGetHandle(Execute := TRUE, ConnectionHdl := hConnection, NodeID := stNodeID, Timeout := T#5S);
    IF fbGetHandle.Done THEN
        hNode := fbGetHandle.NodeHdl;
        fbGetHandle(Execute := FALSE, ConnectionHdl := hConnection, NodeID := stNodeID);
        iStep := 2;
    ELSIF fbGetHandle.Error THEN
        dwErrorID := fbGetHandle.ErrorID;
        fbGetHandle(Execute := FALSE, ConnectionHdl := hConnection, NodeID := stNodeID);
        xError := TRUE; xBusy := FALSE; iStep := 0;
    END_IF
2:  (* one instance per data type, because the value is an IN_OUT of type ANY *)
    CASE eKind OF
    SIM_BOOL:
        IF xWrite THEN
            fbWriteBool(Execute := TRUE, ConnectionHdl := hConnection, NodeHdl := hNode, NodeAddInfo := stInfo, Timeout := T#5S, Variable := xValue);
            xAccessDone := fbWriteBool.Done; xAccessError := fbWriteBool.Error; dwAccessError := fbWriteBool.ErrorID;
            IF xAccessDone OR xAccessError THEN fbWriteBool(Execute := FALSE, Variable := xValue); END_IF
        ELSE
            fbReadBool(Execute := TRUE, ConnectionHdl := hConnection, NodeHdl := hNode, NodeAddInfo := stInfo, Timeout := T#5S, Variable := xValue);
            xAccessDone := fbReadBool.Done; xAccessError := fbReadBool.Error; dwAccessError := fbReadBool.ErrorID;
            IF xAccessDone OR xAccessError THEN fbReadBool(Execute := FALSE, Variable := xValue); END_IF
        END_IF
    SIM_LREAL:
        IF xWrite THEN
            fbWriteReal(Execute := TRUE, ConnectionHdl := hConnection, NodeHdl := hNode, NodeAddInfo := stInfo, Timeout := T#5S, Variable := lrValue);
            xAccessDone := fbWriteReal.Done; xAccessError := fbWriteReal.Error; dwAccessError := fbWriteReal.ErrorID;
            IF xAccessDone OR xAccessError THEN fbWriteReal(Execute := FALSE, Variable := lrValue); END_IF
        ELSE
            fbReadReal(Execute := TRUE, ConnectionHdl := hConnection, NodeHdl := hNode, NodeAddInfo := stInfo, Timeout := T#5S, Variable := lrValue);
            xAccessDone := fbReadReal.Done; xAccessError := fbReadReal.Error; dwAccessError := fbReadReal.ErrorID;
            IF xAccessDone OR xAccessError THEN fbReadReal(Execute := FALSE, Variable := lrValue); END_IF
        END_IF
    SIM_UINT:
        IF xWrite THEN
            fbWriteUint(Execute := TRUE, ConnectionHdl := hConnection, NodeHdl := hNode, NodeAddInfo := stInfo, Timeout := T#5S, Variable := uiValue);
            xAccessDone := fbWriteUint.Done; xAccessError := fbWriteUint.Error; dwAccessError := fbWriteUint.ErrorID;
            IF xAccessDone OR xAccessError THEN fbWriteUint(Execute := FALSE, Variable := uiValue); END_IF
        ELSE
            fbReadUint(Execute := TRUE, ConnectionHdl := hConnection, NodeHdl := hNode, NodeAddInfo := stInfo, Timeout := T#5S, Variable := uiValue);
            xAccessDone := fbReadUint.Done; xAccessError := fbReadUint.Error; dwAccessError := fbReadUint.ErrorID;
            IF xAccessDone OR xAccessError THEN fbReadUint(Execute := FALSE, Variable := uiValue); END_IF
        END_IF
    END_CASE
    IF xAccessDone OR xAccessError THEN
        xError := xAccessError; dwErrorID := dwAccessError;
        iStep := 3;
    END_IF
3:  fbRelease(Execute := TRUE, ConnectionHdl := hConnection, NodeHdl := hNode, Timeout := T#5S);
    IF fbRelease.Done OR fbRelease.Error THEN
        fbRelease(Execute := FALSE, ConnectionHdl := hConnection, NodeHdl := hNode);
        xBusy := FALSE; xDone := NOT xError; iStep := 0;
    END_IF
END_CASE
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_Handshake
VAR_INPUT
    Execute     : BOOL;
    hConnection : DWORD;
    uiNamespace : UINT;
    sAxis       : STRING(31);
    sFb         : STRING(31);                  (* 'Home', 'MoveAbsolute', 'MoveRelative', 'Stop', 'Reset' *)
    iParams     : INT;                         (* 0..5 *)
    asParam     : ARRAY[1..5] OF STRING(31);
    alrParam    : ARRAY[1..5] OF LREAL;
END_VAR
VAR_OUTPUT
    Done           : BOOL;
    Busy           : BOOL;
    Active         : BOOL;
    CommandAborted : BOOL;
    Error          : BOOL;
    ErrorID        : UINT;
END_VAR
VAR
    fbAccess : SIM_Access;
    iPhase   : INT;      (* 0 idle, 1 inputs, 2 Execute low, 3 wait Done low, 4 Execute high, 5 poll, 6 finished *)
    iItem    : INT;
    xLast    : BOOL;
    xStarted : BOOL;
    xValue   : BOOL;
    lrValue  : LREAL;
    uiValue  : UINT;
    axRemote : ARRAY[1..4] OF BOOL;   (* Busy, Done, CommandAborted, Error as read *)
    sBase    : STRING(80);
END_VAR

IF Execute AND NOT xLast THEN
    Done := FALSE; Busy := TRUE; Active := FALSE; CommandAborted := FALSE; Error := FALSE; ErrorID := 0;
    sBase := CONCAT(CONCAT(CONCAT('axis.', sAxis), '.'), CONCAT(sFb, '.'));
    IF iParams > 0 THEN iPhase := 1; ELSE iPhase := 2; END_IF
    iItem := 1; xStarted := FALSE;
ELSIF NOT Execute AND xLast THEN
    Done := FALSE; CommandAborted := FALSE; Error := FALSE; ErrorID := 0;
    IF NOT Busy THEN iPhase := 2; iItem := 1; xStarted := FALSE; END_IF
END_IF
xLast := Execute;

IF iPhase > 0 AND iPhase < 6 THEN
    IF NOT xStarted THEN
        CASE iPhase OF
        1:  fbAccess.sNode := CONCAT(sBase, asParam[iItem]); lrValue := alrParam[iItem];
            fbAccess.eKind := SIM_LREAL; fbAccess.xWrite := TRUE;
        2, 4:
            fbAccess.sNode := CONCAT(sBase, 'Execute'); xValue := iPhase = 4;
            fbAccess.eKind := SIM_BOOL; fbAccess.xWrite := TRUE;
        3:  fbAccess.sNode := CONCAT(sBase, 'Done');
            fbAccess.eKind := SIM_BOOL; fbAccess.xWrite := FALSE;
        5:  CASE iItem OF
            1: fbAccess.sNode := CONCAT(sBase, 'Busy');
            2: fbAccess.sNode := CONCAT(sBase, 'Done');
            3: fbAccess.sNode := CONCAT(sBase, 'CommandAborted');
            4: fbAccess.sNode := CONCAT(sBase, 'Error');
            ELSE fbAccess.sNode := CONCAT(sBase, 'ErrorID');
            END_CASE
            IF iItem <= 4 THEN fbAccess.eKind := SIM_BOOL; ELSE fbAccess.eKind := SIM_UINT; END_IF
            fbAccess.xWrite := FALSE;
        END_CASE
        fbAccess(xExecute := FALSE, hConnection := hConnection, uiNamespace := uiNamespace, xValue := xValue, lrValue := lrValue, uiValue := uiValue);
        xStarted := TRUE;
    END_IF
    fbAccess(xExecute := TRUE, hConnection := hConnection, uiNamespace := uiNamespace, xValue := xValue, lrValue := lrValue, uiValue := uiValue);

    IF fbAccess.xError THEN
        Error := TRUE; ErrorID := 16#FFFF; Busy := FALSE; iPhase := 6;   (* the OPC UA access failed *)
    ELSIF fbAccess.xDone THEN
        xStarted := FALSE;
        CASE iPhase OF
        1:  iItem := iItem + 1;
            IF iItem > iParams THEN iPhase := 2; iItem := 1; END_IF
        2:  IF Busy THEN iPhase := 3; ELSE iPhase := 0; END_IF
        3:  IF NOT xValue THEN iPhase := 4; END_IF   (* outputs are published once a cycle *)
        4:  iPhase := 5; iItem := 1;
        5:  IF iItem <= 4 THEN
                axRemote[iItem] := xValue;
                iItem := iItem + 1;
            ELSE
                ErrorID := uiValue;
                Active := axRemote[1];
                IF axRemote[2] OR axRemote[3] OR axRemote[4] THEN
                    Done := axRemote[2] AND Execute;
                    CommandAborted := axRemote[3] AND Execute;
                    Error := axRemote[4] AND Execute;
                    Busy := FALSE; Active := FALSE;
                    iPhase := 6;
                END_IF
                iItem := 1;
            END_IF
        END_CASE
    END_IF
END_IF
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_MC_Power
VAR_INPUT
    Enable : BOOL; hConnection : DWORD; uiNamespace : UINT; sAxis : STRING(31);
END_VAR
VAR_OUTPUT
    Status : BOOL; Valid : BOOL; Error : BOOL;
END_VAR
VAR
    fbAccess : SIM_Access;
    iStep    : INT;
    xWritten : BOOL;
    xFirst   : BOOL := TRUE;
    xValue   : BOOL;
    lrDummy  : LREAL;
    uiDummy  : UINT;
END_VAR

(* writes Enable when it changes, otherwise reads Power.Status *)
IF iStep = 0 THEN
    fbAccess.eKind := SIM_BOOL;
    IF xFirst OR Enable <> xWritten THEN
        xValue := Enable; xWritten := Enable; xFirst := FALSE;
        fbAccess.sNode := CONCAT(CONCAT('axis.', sAxis), '.Power.Enable'); fbAccess.xWrite := TRUE;
    ELSE
        fbAccess.sNode := CONCAT(CONCAT('axis.', sAxis), '.Power.Status'); fbAccess.xWrite := FALSE;
    END_IF
    fbAccess(xExecute := FALSE, hConnection := hConnection, uiNamespace := uiNamespace, xValue := xValue, lrValue := lrDummy, uiValue := uiDummy);
    iStep := 1;
END_IF
fbAccess(xExecute := TRUE, hConnection := hConnection, uiNamespace := uiNamespace, xValue := xValue, lrValue := lrDummy, uiValue := uiDummy);
IF fbAccess.xDone OR fbAccess.xError THEN
    Error := fbAccess.xError;
    IF fbAccess.xDone AND NOT fbAccess.xWrite THEN Status := xValue; END_IF
    Valid := Status AND NOT Error;
    iStep := 0;
END_IF
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_MC_Home
VAR_INPUT
    Execute : BOOL; hConnection : DWORD; uiNamespace : UINT; sAxis : STRING(31);
END_VAR
VAR_OUTPUT
    Done : BOOL; Busy : BOOL; Active : BOOL; CommandAborted : BOOL; Error : BOOL; ErrorID : UINT;
END_VAR
VAR
    fbCall : SIM_Handshake;
END_VAR
fbCall(Execute := Execute, hConnection := hConnection, uiNamespace := uiNamespace, sAxis := sAxis, sFb := 'Home', iParams := 0);
Done := fbCall.Done; Busy := fbCall.Busy; Active := fbCall.Active;
CommandAborted := fbCall.CommandAborted; Error := fbCall.Error; ErrorID := fbCall.ErrorID;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_MC_MoveAbsolute
VAR_INPUT
    Execute : BOOL; hConnection : DWORD; uiNamespace : UINT; sAxis : STRING(31);
    Position : LREAL; Velocity : LREAL; Acceleration : LREAL; Deceleration : LREAL; Jerk : LREAL;
END_VAR
VAR_OUTPUT
    Done : BOOL; Busy : BOOL; Active : BOOL; CommandAborted : BOOL; Error : BOOL; ErrorID : UINT;
END_VAR
VAR
    fbCall : SIM_Handshake;
END_VAR
fbCall.asParam[1] := 'Position';     fbCall.alrParam[1] := Position;
fbCall.asParam[2] := 'Velocity';     fbCall.alrParam[2] := Velocity;
fbCall.asParam[3] := 'Acceleration'; fbCall.alrParam[3] := Acceleration;
fbCall.asParam[4] := 'Deceleration'; fbCall.alrParam[4] := Deceleration;
fbCall.asParam[5] := 'Jerk';         fbCall.alrParam[5] := Jerk;
fbCall(Execute := Execute, hConnection := hConnection, uiNamespace := uiNamespace, sAxis := sAxis, sFb := 'MoveAbsolute', iParams := 5);
Done := fbCall.Done; Busy := fbCall.Busy; Active := fbCall.Active;
CommandAborted := fbCall.CommandAborted; Error := fbCall.Error; ErrorID := fbCall.ErrorID;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_MC_MoveRelative
VAR_INPUT
    Execute : BOOL; hConnection : DWORD; uiNamespace : UINT; sAxis : STRING(31);
    Distance : LREAL; Velocity : LREAL; Acceleration : LREAL; Deceleration : LREAL; Jerk : LREAL;
END_VAR
VAR_OUTPUT
    Done : BOOL; Busy : BOOL; Active : BOOL; CommandAborted : BOOL; Error : BOOL; ErrorID : UINT;
END_VAR
VAR
    fbCall : SIM_Handshake;
END_VAR
fbCall.asParam[1] := 'Distance';     fbCall.alrParam[1] := Distance;
fbCall.asParam[2] := 'Velocity';     fbCall.alrParam[2] := Velocity;
fbCall.asParam[3] := 'Acceleration'; fbCall.alrParam[3] := Acceleration;
fbCall.asParam[4] := 'Deceleration'; fbCall.alrParam[4] := Deceleration;
fbCall.asParam[5] := 'Jerk';         fbCall.alrParam[5] := Jerk;
fbCall(Execute := Execute, hConnection := hConnection, uiNamespace := uiNamespace, sAxis := sAxis, sFb := 'MoveRelative', iParams := 5);
Done := fbCall.Done; Busy := fbCall.Busy; Active := fbCall.Active;
CommandAborted := fbCall.CommandAborted; Error := fbCall.Error; ErrorID := fbCall.ErrorID;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_MC_Stop
VAR_INPUT
    Execute : BOOL; hConnection : DWORD; uiNamespace : UINT; sAxis : STRING(31);
    Deceleration : LREAL;
END_VAR
VAR_OUTPUT
    Done : BOOL; Busy : BOOL; Active : BOOL; CommandAborted : BOOL; Error : BOOL; ErrorID : UINT;
END_VAR
VAR
    fbCall : SIM_Handshake;
END_VAR
fbCall.asParam[1] := 'Deceleration'; fbCall.alrParam[1] := Deceleration;
fbCall(Execute := Execute, hConnection := hConnection, uiNamespace := uiNamespace, sAxis := sAxis, sFb := 'Stop', iParams := 1);
Done := fbCall.Done; Busy := fbCall.Busy; Active := fbCall.Active;
CommandAborted := fbCall.CommandAborted; Error := fbCall.Error; ErrorID := fbCall.ErrorID;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_MC_Reset
VAR_INPUT
    Execute : BOOL; hConnection : DWORD; uiNamespace : UINT; sAxis : STRING(31);
END_VAR
VAR_OUTPUT
    Done : BOOL; Busy : BOOL; Error : BOOL; ErrorID : UINT;
END_VAR
VAR
    fbCall : SIM_Handshake;
END_VAR
fbCall(Execute := Execute, hConnection := hConnection, uiNamespace := uiNamespace, sAxis := sAxis, sFb := 'Reset', iParams := 0);
Done := fbCall.Done; Busy := fbCall.Busy; Error := fbCall.Error; ErrorID := fbCall.ErrorID;
END_FUNCTION_BLOCK

PROGRAM PRG_SimDemo
VAR
    fbConnect    : UA_Connect;
    fbNamespace  : UA_NamespaceGetIndex;
    fbDisconnect : UA_Disconnect;
    stSession    : UASessionConnectInfo;   (* CHECK: type and member names *)
    hConnection  : DWORD;
    uiNamespace  : UINT;
    fbPower      : SIM_MC_Power;
    fbHome       : SIM_MC_Home;
    fbMove       : SIM_MC_MoveAbsolute;
    fbStop       : SIM_MC_Stop;
    tonMoving    : TON;
    iStep        : INT;
    sAxis        : STRING(31) := 'x';
END_VAR

CASE iStep OF
0:  stSession.ApplicationName := 'CODESYS sim client';
    stSession.SecurityMsgMode := UASecurityMsgMode.UASecurityMsgMode_None;            (* CHECK *)
    stSession.SecurityPolicyUri := 'http://opcfoundation.org/UA/SecurityPolicy#None'; (* CHECK *)
    fbConnect(Execute := TRUE, ServerEndpointUrl := 'opc.tcp://192.168.0.10:4840', SessionConnectInfo := stSession, Timeout := T#5S);
    IF fbConnect.Done THEN hConnection := fbConnect.ConnectionHdl; fbConnect(Execute := FALSE); iStep := 10;
    ELSIF fbConnect.Error THEN fbConnect(Execute := FALSE); iStep := 900; END_IF
10: fbNamespace(Execute := TRUE, ConnectionHdl := hConnection, NamespaceUri := 'urn:kinematics-simulation', Timeout := T#5S);
    IF fbNamespace.Done THEN uiNamespace := fbNamespace.NamespaceIndex; fbNamespace(Execute := FALSE); iStep := 20;
    ELSIF fbNamespace.Error THEN fbNamespace(Execute := FALSE); iStep := 900; END_IF
20: fbPower.Enable := TRUE;
    IF fbPower.Status THEN iStep := 30; END_IF
30: fbHome.Execute := TRUE;
    IF fbHome.Done THEN fbHome.Execute := FALSE; iStep := 40;
    ELSIF fbHome.Error THEN iStep := 900; END_IF
40: fbMove.Position := 0.6; fbMove.Velocity := 0.5; fbMove.Acceleration := 5.0; fbMove.Deceleration := 5.0; fbMove.Jerk := 100.0;
    fbMove.Execute := TRUE;
    tonMoving(IN := fbMove.Busy, PT := T#500MS);
    IF tonMoving.Q OR fbMove.Done THEN iStep := 50;
    ELSIF fbMove.Error THEN iStep := 900; END_IF
50: fbStop.Deceleration := 2.0;
    fbStop.Execute := TRUE;
    IF fbStop.Done THEN fbStop.Execute := FALSE; fbMove.Execute := FALSE; iStep := 60;
    ELSIF fbStop.Error THEN iStep := 900; END_IF
60: fbPower.Enable := FALSE;
    IF NOT fbPower.Status THEN iStep := 70; END_IF
70: fbDisconnect(Execute := TRUE, ConnectionHdl := hConnection, Timeout := T#5S);
    IF fbDisconnect.Done OR fbDisconnect.Error THEN fbDisconnect(Execute := FALSE); iStep := 100; END_IF
100: ;
900: ;
END_CASE

IF iStep >= 20 AND iStep < 70 THEN
    fbPower(hConnection := hConnection, uiNamespace := uiNamespace, sAxis := sAxis);
    fbHome(hConnection := hConnection, uiNamespace := uiNamespace, sAxis := sAxis);
    fbMove(hConnection := hConnection, uiNamespace := uiNamespace, sAxis := sAxis);
    fbStop(hConnection := hConnection, uiNamespace := uiNamespace, sAxis := sAxis);
END_IF
END_PROGRAM

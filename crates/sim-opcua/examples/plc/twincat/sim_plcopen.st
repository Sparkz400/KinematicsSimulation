(*
    PLCopen motion over OPC UA to the KinematicsSimulation link (sim-opcua), for TwinCAT 3.
    Library: Tc3_PLCopen_OpcUa (add it in the PLC project's References).

    NOT COMPILED: this file was written against the Tc3_PLCopen_OpcUa documentation and has not been built in
    TwinCAT. Check the names marked "CHECK" against the library version you have.

    Structure:
      SIM_Access          one OPC UA read or write of one variable (get handle, read/write, release handle)
      SIM_Handshake       the PLCopen handshake of one function block on the simulator: inputs, Execute low,
                          wait for Done low, Execute high, poll Busy/Done/CommandAborted/Error/ErrorID
      SIM_MC_Power, SIM_MC_Home, SIM_MC_MoveAbsolute, SIM_MC_MoveRelative, SIM_MC_Stop, SIM_MC_Reset
                          thin wrappers with PLCopen inputs and outputs
      PRG_SimDemo         connect, power, home, move, stop, disconnect

    Node ids are strings in the namespace "urn:kinematics-simulation", e.g. 'axis.x.MoveAbsolute.Execute'.
    Getting a handle for every access keeps the code short and costs three round trips; cache the handles in
    production code.
*)

TYPE E_SimKind : (SIM_BOOL, SIM_LREAL, SIM_UINT); END_TYPE

FUNCTION_BLOCK SIM_Access
VAR_INPUT
    bExecute    : BOOL;            (* rising edge starts the access *)
    hConnection : DWORD;
    nNamespace  : UINT;
    sNode       : STRING(255);     (* e.g. 'axis.x.Power.Enable' *)
    bWrite      : BOOL;            (* TRUE: write the value at pData; FALSE: read into it *)
    pData       : PVOID;
    cbData      : UDINT;
END_VAR
VAR_OUTPUT
    bDone    : BOOL;
    bBusy    : BOOL;
    bError   : BOOL;
    nErrorID : DWORD;
END_VAR
VAR
    fbGetHandle : UA_NodeGetHandle;
    fbRead      : UA_Read;
    fbWrite     : UA_Write;
    fbRelease   : UA_NodeReleaseHandle;
    stNodeID    : ST_UANodeID;
    stInfo      : ST_UANodeAdditionalInfo;   (* CHECK: member name of the attribute id *)
    hNode       : DWORD;
    nStep       : INT;
    bLast       : BOOL;
END_VAR

IF bExecute AND NOT bLast THEN
    bDone := FALSE; bError := FALSE; nErrorID := 0; bBusy := TRUE;
    stNodeID.nNamespaceIndex := nNamespace;
    stNodeID.sIdentifier := sNode;
    stNodeID.eIdentifierType := eUAIdentifierType.String;
    stInfo.nAttributeID := 13;   (* the Value attribute *)
    nStep := 1;
END_IF
bLast := bExecute;

CASE nStep OF
1:  fbGetHandle(Execute := TRUE, ConnectionHdl := hConnection, NodeID := stNodeID);
    IF fbGetHandle.Done THEN
        hNode := fbGetHandle.NodeHdl;
        fbGetHandle(Execute := FALSE);
        nStep := 2;
    ELSIF fbGetHandle.Error THEN
        nErrorID := fbGetHandle.ErrorID;
        fbGetHandle(Execute := FALSE);
        bError := TRUE; bBusy := FALSE; nStep := 0;
    END_IF
2:  IF bWrite THEN
        fbWrite(Execute := TRUE, ConnectionHdl := hConnection, NodeHdl := hNode, stNodeAddInfo := stInfo,
                cbData := cbData, pVariable := pData);
        IF fbWrite.Done OR fbWrite.Error THEN
            bError := fbWrite.Error; nErrorID := fbWrite.ErrorID;
            fbWrite(Execute := FALSE);
            nStep := 3;
        END_IF
    ELSE
        fbRead(Execute := TRUE, ConnectionHdl := hConnection, NodeHdl := hNode, stNodeAddInfo := stInfo,
               cbData := cbData, pVariable := pData);
        IF fbRead.Done OR fbRead.Error THEN
            bError := fbRead.Error; nErrorID := fbRead.ErrorID;
            fbRead(Execute := FALSE);
            nStep := 3;
        END_IF
    END_IF
3:  fbRelease(Execute := TRUE, ConnectionHdl := hConnection, NodeHdl := hNode);
    IF fbRelease.Done OR fbRelease.Error THEN
        fbRelease(Execute := FALSE);
        bBusy := FALSE; bDone := NOT bError; nStep := 0;
    END_IF
END_CASE
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_Handshake
VAR_INPUT
    Execute     : BOOL;
    hConnection : DWORD;
    nNamespace  : UINT;
    sAxis       : STRING(31);                  (* axis id, e.g. 'x' *)
    sFb         : STRING(31);                  (* 'Home', 'MoveAbsolute', 'MoveRelative', 'Stop', 'Reset' *)
    nParams     : INT;                         (* how many of the inputs below to write, 0..5 *)
    asParam     : ARRAY[1..5] OF STRING(31);   (* e.g. 'Position' *)
    afParam     : ARRAY[1..5] OF LREAL;
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
    fbAccess  : SIM_Access;
    nPhase    : INT;      (* 0 idle, 1 inputs, 2 Execute low, 3 wait Done low, 4 Execute high, 5 poll, 6 finished *)
    nItem     : INT;
    bLast     : BOOL;
    bStarted  : BOOL;     (* fbAccess was triggered for the current item *)
    bValue    : BOOL;
    fValue    : LREAL;
    nValue    : UINT;
    bRemote   : ARRAY[1..4] OF BOOL;   (* Busy, Done, CommandAborted, Error as read *)
    sBase     : STRING(80);
END_VAR

IF Execute AND NOT bLast THEN
    Done := FALSE; Busy := TRUE; Active := FALSE; CommandAborted := FALSE; Error := FALSE; ErrorID := 0;
    sBase := CONCAT(CONCAT(CONCAT('axis.', sAxis), '.'), CONCAT(sFb, '.'));
    IF nParams > 0 THEN nPhase := 1; ELSE nPhase := 2; END_IF
    nItem := 1; bStarted := FALSE;
ELSIF NOT Execute AND bLast THEN
    (* PLCopen: the outputs fall with Execute; the simulator's Execute goes low with it *)
    Done := FALSE; CommandAborted := FALSE; Error := FALSE; ErrorID := 0;
    IF NOT Busy THEN nPhase := 2; nItem := 1; bStarted := FALSE; END_IF
END_IF
bLast := Execute;

IF nPhase > 0 AND nPhase < 6 THEN
    IF NOT bStarted THEN
        (* the access of this phase and item *)
        fbAccess.hConnection := hConnection;
        fbAccess.nNamespace := nNamespace;
        CASE nPhase OF
        1:  fbAccess.sNode := CONCAT(sBase, asParam[nItem]); fValue := afParam[nItem];
            fbAccess.bWrite := TRUE; fbAccess.pData := ADR(fValue); fbAccess.cbData := SIZEOF(fValue);
        2, 4:
            fbAccess.sNode := CONCAT(sBase, 'Execute'); bValue := nPhase = 4;
            fbAccess.bWrite := TRUE; fbAccess.pData := ADR(bValue); fbAccess.cbData := SIZEOF(bValue);
        3:  fbAccess.sNode := CONCAT(sBase, 'Done');
            fbAccess.bWrite := FALSE; fbAccess.pData := ADR(bValue); fbAccess.cbData := SIZEOF(bValue);
        5:  IF nItem <= 4 THEN
                CASE nItem OF
                1: fbAccess.sNode := CONCAT(sBase, 'Busy');
                2: fbAccess.sNode := CONCAT(sBase, 'Done');
                3: fbAccess.sNode := CONCAT(sBase, 'CommandAborted');
                4: fbAccess.sNode := CONCAT(sBase, 'Error');
                END_CASE
                fbAccess.pData := ADR(bValue); fbAccess.cbData := SIZEOF(bValue);
            ELSE
                fbAccess.sNode := CONCAT(sBase, 'ErrorID');
                fbAccess.pData := ADR(nValue); fbAccess.cbData := SIZEOF(nValue);
            END_IF
            fbAccess.bWrite := FALSE;
        END_CASE
        fbAccess(bExecute := FALSE);
        fbAccess(bExecute := TRUE);
        bStarted := TRUE;
    ELSE
        fbAccess(bExecute := TRUE);
    END_IF

    IF fbAccess.bError THEN
        Error := TRUE; ErrorID := 16#FFFF; Busy := FALSE; nPhase := 6;   (* the OPC UA access failed *)
    ELSIF fbAccess.bDone THEN
        bStarted := FALSE;
        CASE nPhase OF
        1:  nItem := nItem + 1;
            IF nItem > nParams THEN nPhase := 2; nItem := 1; END_IF
        2:  IF Busy THEN nPhase := 3; ELSE nPhase := 0; END_IF   (* after the falling edge: done *)
        3:  IF NOT bValue THEN nPhase := 4; END_IF                (* outputs are published once a cycle *)
        4:  nPhase := 5; nItem := 1;
        5:  IF nItem <= 4 THEN
                bRemote[nItem] := bValue;
                nItem := nItem + 1;
            ELSE
                ErrorID := nValue;
                Active := bRemote[1];
                IF bRemote[2] OR bRemote[3] OR bRemote[4] THEN
                    Done := bRemote[2] AND Execute;
                    CommandAborted := bRemote[3] AND Execute;
                    Error := bRemote[4] AND Execute;
                    Busy := FALSE; Active := FALSE;
                    nPhase := 6;
                END_IF
                nItem := 1;
            END_IF
        END_CASE
    END_IF
END_IF
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_MC_Power
VAR_INPUT
    Enable      : BOOL;
    hConnection : DWORD;
    nNamespace  : UINT;
    sAxis       : STRING(31);
END_VAR
VAR_OUTPUT
    Status : BOOL;
    Valid  : BOOL;
    Error  : BOOL;
END_VAR
VAR
    fbAccess : SIM_Access;
    nStep    : INT;
    bWritten : BOOL;   (* the Enable last written *)
    bValue   : BOOL;
    bFirst   : BOOL := TRUE;
END_VAR

(* writes Enable when it changes, otherwise reads Power.Status *)
CASE nStep OF
0:  fbAccess.hConnection := hConnection; fbAccess.nNamespace := nNamespace;
    IF bFirst OR Enable <> bWritten THEN
        bValue := Enable; bWritten := Enable; bFirst := FALSE;
        fbAccess.sNode := CONCAT(CONCAT('axis.', sAxis), '.Power.Enable');
        fbAccess.bWrite := TRUE;
    ELSE
        fbAccess.sNode := CONCAT(CONCAT('axis.', sAxis), '.Power.Status');
        fbAccess.bWrite := FALSE;
    END_IF
    fbAccess.pData := ADR(bValue); fbAccess.cbData := SIZEOF(bValue);
    fbAccess(bExecute := FALSE);
    fbAccess(bExecute := TRUE);
    nStep := 1;
1:  fbAccess(bExecute := TRUE);
    IF fbAccess.bDone OR fbAccess.bError THEN
        Error := fbAccess.bError;
        IF fbAccess.bDone AND NOT fbAccess.bWrite THEN Status := bValue; END_IF
        Valid := Status AND NOT Error;
        nStep := 0;
    END_IF
END_CASE
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_MC_Home
VAR_INPUT
    Execute : BOOL; hConnection : DWORD; nNamespace : UINT; sAxis : STRING(31);
END_VAR
VAR_OUTPUT
    Done : BOOL; Busy : BOOL; Active : BOOL; CommandAborted : BOOL; Error : BOOL; ErrorID : UINT;
END_VAR
VAR
    fbCall : SIM_Handshake;
END_VAR
fbCall(Execute := Execute, hConnection := hConnection, nNamespace := nNamespace, sAxis := sAxis, sFb := 'Home', nParams := 0);
Done := fbCall.Done; Busy := fbCall.Busy; Active := fbCall.Active;
CommandAborted := fbCall.CommandAborted; Error := fbCall.Error; ErrorID := fbCall.ErrorID;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_MC_MoveAbsolute
VAR_INPUT
    Execute : BOOL; hConnection : DWORD; nNamespace : UINT; sAxis : STRING(31);
    Position : LREAL; Velocity : LREAL; Acceleration : LREAL; Deceleration : LREAL; Jerk : LREAL;
END_VAR
VAR_OUTPUT
    Done : BOOL; Busy : BOOL; Active : BOOL; CommandAborted : BOOL; Error : BOOL; ErrorID : UINT;
END_VAR
VAR
    fbCall : SIM_Handshake;
END_VAR
fbCall.asParam[1] := 'Position';     fbCall.afParam[1] := Position;
fbCall.asParam[2] := 'Velocity';     fbCall.afParam[2] := Velocity;
fbCall.asParam[3] := 'Acceleration'; fbCall.afParam[3] := Acceleration;
fbCall.asParam[4] := 'Deceleration'; fbCall.afParam[4] := Deceleration;
fbCall.asParam[5] := 'Jerk';         fbCall.afParam[5] := Jerk;
fbCall(Execute := Execute, hConnection := hConnection, nNamespace := nNamespace, sAxis := sAxis, sFb := 'MoveAbsolute', nParams := 5);
Done := fbCall.Done; Busy := fbCall.Busy; Active := fbCall.Active;
CommandAborted := fbCall.CommandAborted; Error := fbCall.Error; ErrorID := fbCall.ErrorID;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_MC_MoveRelative
VAR_INPUT
    Execute : BOOL; hConnection : DWORD; nNamespace : UINT; sAxis : STRING(31);
    Distance : LREAL; Velocity : LREAL; Acceleration : LREAL; Deceleration : LREAL; Jerk : LREAL;
END_VAR
VAR_OUTPUT
    Done : BOOL; Busy : BOOL; Active : BOOL; CommandAborted : BOOL; Error : BOOL; ErrorID : UINT;
END_VAR
VAR
    fbCall : SIM_Handshake;
END_VAR
fbCall.asParam[1] := 'Distance';     fbCall.afParam[1] := Distance;
fbCall.asParam[2] := 'Velocity';     fbCall.afParam[2] := Velocity;
fbCall.asParam[3] := 'Acceleration'; fbCall.afParam[3] := Acceleration;
fbCall.asParam[4] := 'Deceleration'; fbCall.afParam[4] := Deceleration;
fbCall.asParam[5] := 'Jerk';         fbCall.afParam[5] := Jerk;
fbCall(Execute := Execute, hConnection := hConnection, nNamespace := nNamespace, sAxis := sAxis, sFb := 'MoveRelative', nParams := 5);
Done := fbCall.Done; Busy := fbCall.Busy; Active := fbCall.Active;
CommandAborted := fbCall.CommandAborted; Error := fbCall.Error; ErrorID := fbCall.ErrorID;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_MC_Stop
VAR_INPUT
    Execute : BOOL; hConnection : DWORD; nNamespace : UINT; sAxis : STRING(31);
    Deceleration : LREAL;
END_VAR
VAR_OUTPUT
    Done : BOOL; Busy : BOOL; Active : BOOL; CommandAborted : BOOL; Error : BOOL; ErrorID : UINT;
END_VAR
VAR
    fbCall : SIM_Handshake;
END_VAR
fbCall.asParam[1] := 'Deceleration'; fbCall.afParam[1] := Deceleration;
fbCall(Execute := Execute, hConnection := hConnection, nNamespace := nNamespace, sAxis := sAxis, sFb := 'Stop', nParams := 1);
Done := fbCall.Done; Busy := fbCall.Busy; Active := fbCall.Active;
CommandAborted := fbCall.CommandAborted; Error := fbCall.Error; ErrorID := fbCall.ErrorID;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_MC_Reset
VAR_INPUT
    Execute : BOOL; hConnection : DWORD; nNamespace : UINT; sAxis : STRING(31);
END_VAR
VAR_OUTPUT
    Done : BOOL; Busy : BOOL; Error : BOOL; ErrorID : UINT;
END_VAR
VAR
    fbCall : SIM_Handshake;
END_VAR
fbCall(Execute := Execute, hConnection := hConnection, nNamespace := nNamespace, sAxis := sAxis, sFb := 'Reset', nParams := 0);
Done := fbCall.Done; Busy := fbCall.Busy; Error := fbCall.Error; ErrorID := fbCall.ErrorID;
END_FUNCTION_BLOCK

PROGRAM PRG_SimDemo
VAR
    fbConnect    : UA_Connect;
    fbNamespace  : UA_GetNamespaceIndex;
    fbDisconnect : UA_Disconnect;
    stSession    : ST_UASessionConnectInfo;
    hConnection  : DWORD;
    nNamespace   : UINT;
    fbPower      : SIM_MC_Power;
    fbHome       : SIM_MC_Home;
    fbMove       : SIM_MC_MoveAbsolute;
    fbStop       : SIM_MC_Stop;
    tonMoving    : TON;
    nStep        : INT;
    sAxis        : STRING(31) := 'x';    (* an axis id of the simulated machine *)
END_VAR

CASE nStep OF
0:  (* connect without security; the link offers SecurityPolicy None with anonymous login *)
    stSession.sApplicationName := 'TwinCAT sim client';
    stSession.eSecurityMode := eUASecurityMsgMode.None;
    stSession.eSecurityPolicyUri := eUASecurityPolicyUri.None;
    stSession.eTransportProfileUri := eUATransportProfileUri.UATcp;
    stSession.nSessionTimeout := T#60S;
    fbConnect(Execute := TRUE, ServerURL := 'opc.tcp://192.168.0.10:4840', SessionConnectInfo := stSession, Timeout := T#5S);
    IF fbConnect.Done THEN hConnection := fbConnect.ConnectionHdl; fbConnect(Execute := FALSE); nStep := 10;
    ELSIF fbConnect.Error THEN fbConnect(Execute := FALSE); nStep := 900; END_IF
10: fbNamespace(Execute := TRUE, ConnectionHdl := hConnection, NamespaceUri := 'urn:kinematics-simulation', Timeout := T#5S);
    IF fbNamespace.Done THEN nNamespace := fbNamespace.NamespaceIndex; fbNamespace(Execute := FALSE); nStep := 20;
    ELSIF fbNamespace.Error THEN fbNamespace(Execute := FALSE); nStep := 900; END_IF
20: (* power on *)
    fbPower.Enable := TRUE;
    IF fbPower.Status THEN nStep := 30; END_IF
30: (* home *)
    fbHome.Execute := TRUE;
    IF fbHome.Done THEN fbHome.Execute := FALSE; nStep := 40;
    ELSIF fbHome.Error THEN nStep := 900; END_IF
40: (* move absolute, stopped after 0.5 s *)
    fbMove.Position := 0.6; fbMove.Velocity := 0.5; fbMove.Acceleration := 5.0; fbMove.Deceleration := 5.0; fbMove.Jerk := 100.0;
    fbMove.Execute := TRUE;
    tonMoving(IN := fbMove.Busy, PT := T#500MS);
    IF tonMoving.Q OR fbMove.Done THEN nStep := 50;
    ELSIF fbMove.Error THEN nStep := 900; END_IF
50: fbStop.Deceleration := 2.0;
    fbStop.Execute := TRUE;
    IF fbStop.Done THEN fbStop.Execute := FALSE; fbMove.Execute := FALSE; nStep := 60;
    ELSIF fbStop.Error THEN nStep := 900; END_IF
60: (* power off and disconnect *)
    fbPower.Enable := FALSE;
    IF NOT fbPower.Status THEN nStep := 70; END_IF
70: fbDisconnect(Execute := TRUE, ConnectionHdl := hConnection, Timeout := T#5S);
    IF fbDisconnect.Done OR fbDisconnect.Error THEN fbDisconnect(Execute := FALSE); nStep := 100; END_IF
100: ; (* finished *)
900: ; (* error: look at the blocks' ErrorID *)
END_CASE

IF nStep >= 20 AND nStep < 70 THEN
    fbPower(hConnection := hConnection, nNamespace := nNamespace, sAxis := sAxis);
    fbHome(hConnection := hConnection, nNamespace := nNamespace, sAxis := sAxis);
    fbMove(hConnection := hConnection, nNamespace := nNamespace, sAxis := sAxis);
    fbStop(hConnection := hConnection, nNamespace := nNamespace, sAxis := sAxis);
END_IF
END_PROGRAM

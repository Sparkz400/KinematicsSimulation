(*
    PLCopen synchronised motion over OPC UA to the KinematicsSimulation link (sim-opcua), for TwinCAT 3:
    MC_GearIn, MC_GearOut, MC_CamIn and MC_CamOut on a slave axis, its master another axis of the machine.
    Library: Tc3_PLCopen_OpcUa. Needs SIM_Access and SIM_Handshake from sim_plcopen.st in the same project.

    NOT COMPILED: TwinCAT needs a Beckhoff account to download, so this file was not built. It uses only the blocks
    of sim_plcopen.st (whose names were checked there) and plain IEC 61131-3. CHECK: the size TF6100's UA_Write
    takes for a STRING (here LEN + 1, the text and its terminating zero); the link reads the text up to its end.

    Structure:
      SIM_WriteText        one write of a text input (GearIn.Master, CamIn.Master, CamIn.Table)
      SIM_SyncHandshake    the handshake of MC_GearIn or MC_CamIn: inputs and texts, Execute low, Execute high, then
                           poll Busy, InGear/InSync, CommandAborted, Error and ErrorID for as long as the block is busy:
                           unlike a move it stays busy while the slave is coupled
      SIM_MC_GearIn, SIM_MC_CamIn, SIM_MC_GearOut, SIM_MC_CamOut   thin wrappers with PLCopen inputs and outputs
      PRG_SimSyncDemo      x moves, y gears in at 2:1, gears out, cams in on a 3-4-5 table, cams out, stops

    The master is named by its axis id ('x'); an empty CamIn.Master with CamTableID 1 runs the slave on the machine's
    own electronic cam and its virtual master (the crank and cam machines). See docs/plc.md.
*)

FUNCTION_BLOCK SIM_WriteText
VAR_INPUT
    bExecute    : BOOL;
    hConnection : DWORD;
    nNamespace  : UINT;
    sNode       : STRING(255);
    sText       : STRING(1023);
END_VAR
VAR_OUTPUT
    bDone : BOOL; bBusy : BOOL; bError : BOOL; nErrorID : DWORD;
END_VAR
VAR
    fbAccess : SIM_Access;
END_VAR
fbAccess(bExecute := bExecute, hConnection := hConnection, nNamespace := nNamespace, sNode := sNode, bWrite := TRUE,
         pData := ADR(sText), cbData := INT_TO_UDINT(LEN(sText)) + 1);
bDone := fbAccess.bDone; bBusy := fbAccess.bBusy; bError := fbAccess.bError; nErrorID := fbAccess.nErrorID;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_SyncHandshake
VAR_INPUT
    Execute     : BOOL;
    hConnection : DWORD;
    nNamespace  : UINT;
    sAxis       : STRING(31);                  (* the slave's axis id *)
    sFb         : STRING(31);                  (* 'GearIn' or 'CamIn' *)
    sMaster     : STRING(31);                  (* the master's axis id; '' for the machine's virtual cam master *)
    sTable      : STRING(1023);                (* CamIn only: the cam table, '' to leave it as it is *)
    nParams     : INT;                         (* 0..6 *)
    asParam     : ARRAY[1..6] OF STRING(31);
    afParam     : ARRAY[1..6] OF LREAL;
END_VAR
VAR_OUTPUT
    InSync         : BOOL;     (* InGear of MC_GearIn, InSync of MC_CamIn *)
    Busy           : BOOL;
    Active         : BOOL;
    CommandAborted : BOOL;
    Error          : BOOL;
    ErrorID        : UINT;
END_VAR
VAR
    fbAccess : SIM_Access;
    fbText   : SIM_WriteText;
    nPhase   : INT;      (* 0 idle, 1 inputs, 2 Master, 3 Table, 4 Execute low, 5 Execute high, 6 poll, 7 finished *)
    nItem    : INT;
    bLast    : BOOL;
    bStarted : BOOL;
    bValue   : BOOL;
    fValue   : LREAL;
    nValue   : UINT;
    bRemote  : ARRAY[1..4] OF BOOL;   (* Busy, InGear/InSync, CommandAborted, Error as read *)
    sBase    : STRING(80);
    sSync    : STRING(31);
END_VAR

IF Execute AND NOT bLast THEN
    InSync := FALSE; Busy := TRUE; Active := FALSE; CommandAborted := FALSE; Error := FALSE; ErrorID := 0;
    sBase := CONCAT(CONCAT(CONCAT('axis.', sAxis), '.'), CONCAT(sFb, '.'));
    IF sFb = 'GearIn' THEN sSync := 'InGear'; ELSE sSync := 'InSync'; END_IF
    IF nParams > 0 THEN nPhase := 1; ELSE nPhase := 2; END_IF
    nItem := 1; bStarted := FALSE;
ELSIF NOT Execute AND bLast THEN
    (* the outputs fall with Execute; a coupled block stays coupled until GearOut/CamOut or another block *)
    InSync := FALSE; CommandAborted := FALSE; Error := FALSE; ErrorID := 0;
    nPhase := 4; bStarted := FALSE;
END_IF
bLast := Execute;

IF nPhase = 2 OR nPhase = 3 THEN
    (* the texts: the master, then (CamIn) the table *)
    IF nPhase = 3 AND (sFb <> 'CamIn' OR LEN(sTable) = 0) THEN
        nPhase := 4;
    ELSE
        IF NOT bStarted THEN
            fbText.hConnection := hConnection; fbText.nNamespace := nNamespace;
            IF nPhase = 2 THEN
                fbText.sNode := CONCAT(sBase, 'Master'); fbText.sText := sMaster;
            ELSE
                fbText.sNode := CONCAT(sBase, 'Table'); fbText.sText := sTable;
            END_IF
            fbText(bExecute := FALSE);
            bStarted := TRUE;
        END_IF
        fbText(bExecute := TRUE);
        IF fbText.bError THEN
            Error := TRUE; ErrorID := 16#FFFF; Busy := FALSE; nPhase := 7;
        ELSIF fbText.bDone THEN
            bStarted := FALSE; nPhase := nPhase + 1;
        END_IF
    END_IF
ELSIF nPhase > 0 AND nPhase < 7 THEN
    IF NOT bStarted THEN
        fbAccess.hConnection := hConnection;
        fbAccess.nNamespace := nNamespace;
        CASE nPhase OF
        1:  fbAccess.sNode := CONCAT(sBase, asParam[nItem]); fValue := afParam[nItem];
            fbAccess.bWrite := TRUE; fbAccess.pData := ADR(fValue); fbAccess.cbData := SIZEOF(fValue);
        4, 5:
            fbAccess.sNode := CONCAT(sBase, 'Execute'); bValue := nPhase = 5 AND Execute;
            fbAccess.bWrite := TRUE; fbAccess.pData := ADR(bValue); fbAccess.cbData := SIZEOF(bValue);
        6:  IF nItem <= 4 THEN
                CASE nItem OF
                1: fbAccess.sNode := CONCAT(sBase, 'Busy');
                2: fbAccess.sNode := CONCAT(sBase, sSync);
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
        Error := TRUE; ErrorID := 16#FFFF; Busy := FALSE; nPhase := 7;   (* the OPC UA access failed *)
    ELSIF fbAccess.bDone THEN
        bStarted := FALSE;
        CASE nPhase OF
        1:  nItem := nItem + 1;
            IF nItem > nParams THEN nPhase := 2; nItem := 1; END_IF
        4:  IF Execute THEN nPhase := 5; ELSE nPhase := 0; Busy := FALSE; Active := FALSE; END_IF
        5:  nPhase := 6; nItem := 1;
        6:  IF nItem <= 4 THEN
                bRemote[nItem] := bValue;
                nItem := nItem + 1;
            ELSE
                ErrorID := nValue;
                Busy := bRemote[1]; Active := bRemote[1];
                InSync := bRemote[2] AND Execute;
                CommandAborted := bRemote[3] AND Execute;
                Error := bRemote[4] AND Execute;
                (* coupled, the block stays busy: poll on; it is over once it is no longer busy *)
                IF NOT Busy THEN nPhase := 7; END_IF
                nItem := 1;
            END_IF
        END_CASE
    END_IF
END_IF
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_MC_GearIn
VAR_INPUT
    Execute : BOOL; hConnection : DWORD; nNamespace : UINT;
    sMaster : STRING(31); sSlave : STRING(31);
    RatioNumerator : LREAL; RatioDenominator : LREAL; Acceleration : LREAL; Deceleration : LREAL; Jerk : LREAL;
END_VAR
VAR_OUTPUT
    InGear : BOOL; Busy : BOOL; Active : BOOL; CommandAborted : BOOL; Error : BOOL; ErrorID : UINT;
END_VAR
VAR
    fbCall : SIM_SyncHandshake;
END_VAR
fbCall.asParam[1] := 'RatioNumerator';   fbCall.afParam[1] := RatioNumerator;
fbCall.asParam[2] := 'RatioDenominator'; fbCall.afParam[2] := RatioDenominator;
fbCall.asParam[3] := 'Acceleration';     fbCall.afParam[3] := Acceleration;
fbCall.asParam[4] := 'Deceleration';     fbCall.afParam[4] := Deceleration;
fbCall.asParam[5] := 'Jerk';             fbCall.afParam[5] := Jerk;
fbCall(Execute := Execute, hConnection := hConnection, nNamespace := nNamespace, sAxis := sSlave, sFb := 'GearIn',
       sMaster := sMaster, sTable := '', nParams := 5);
InGear := fbCall.InSync; Busy := fbCall.Busy; Active := fbCall.Active;
CommandAborted := fbCall.CommandAborted; Error := fbCall.Error; ErrorID := fbCall.ErrorID;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_MC_CamIn
VAR_INPUT
    Execute : BOOL; hConnection : DWORD; nNamespace : UINT;
    sMaster : STRING(31); sSlave : STRING(31);
    MasterOffset : LREAL; SlaveOffset : LREAL; MasterScaling : LREAL := 1.0; SlaveScaling : LREAL := 1.0;
    StartMode : LREAL := 1.0;    (* 0 absolute, 1 relative *)
    CamTableID : LREAL;          (* 0 the table below, 1 the machine's own cam *)
    sTable : STRING(1023);       (* lines 'master span, slave rise, law' or 'master, slave[, slope]' *)
END_VAR
VAR_OUTPUT
    InSync : BOOL; Busy : BOOL; Active : BOOL; CommandAborted : BOOL; Error : BOOL; ErrorID : UINT;
END_VAR
VAR
    fbCall : SIM_SyncHandshake;
END_VAR
fbCall.asParam[1] := 'MasterOffset';  fbCall.afParam[1] := MasterOffset;
fbCall.asParam[2] := 'SlaveOffset';   fbCall.afParam[2] := SlaveOffset;
fbCall.asParam[3] := 'MasterScaling'; fbCall.afParam[3] := MasterScaling;
fbCall.asParam[4] := 'SlaveScaling';  fbCall.afParam[4] := SlaveScaling;
fbCall.asParam[5] := 'StartMode';     fbCall.afParam[5] := StartMode;
fbCall.asParam[6] := 'CamTableID';    fbCall.afParam[6] := CamTableID;
fbCall(Execute := Execute, hConnection := hConnection, nNamespace := nNamespace, sAxis := sSlave, sFb := 'CamIn',
       sMaster := sMaster, sTable := sTable, nParams := 6);
InSync := fbCall.InSync; Busy := fbCall.Busy; Active := fbCall.Active;
CommandAborted := fbCall.CommandAborted; Error := fbCall.Error; ErrorID := fbCall.ErrorID;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_MC_GearOut
VAR_INPUT
    Execute : BOOL; hConnection : DWORD; nNamespace : UINT; sSlave : STRING(31);
END_VAR
VAR_OUTPUT
    Done : BOOL; Busy : BOOL; Error : BOOL; ErrorID : UINT;
END_VAR
VAR
    fbCall : SIM_Handshake;
END_VAR
fbCall(Execute := Execute, hConnection := hConnection, nNamespace := nNamespace, sAxis := sSlave, sFb := 'GearOut', nParams := 0);
Done := fbCall.Done; Busy := fbCall.Busy; Error := fbCall.Error; ErrorID := fbCall.ErrorID;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_MC_CamOut
VAR_INPUT
    Execute : BOOL; hConnection : DWORD; nNamespace : UINT; sSlave : STRING(31);
END_VAR
VAR_OUTPUT
    Done : BOOL; Busy : BOOL; Error : BOOL; ErrorID : UINT;
END_VAR
VAR
    fbCall : SIM_Handshake;
END_VAR
fbCall(Execute := Execute, hConnection := hConnection, nNamespace := nNamespace, sAxis := sSlave, sFb := 'CamOut', nParams := 0);
Done := fbCall.Done; Busy := fbCall.Busy; Error := fbCall.Error; ErrorID := fbCall.ErrorID;
END_FUNCTION_BLOCK

PROGRAM PRG_SimSyncDemo
VAR
    fbConnect    : UA_Connect;
    fbNamespace  : UA_GetNamespaceIndex;
    fbDisconnect : UA_Disconnect;
    stSession    : ST_UASessionConnectInfo;
    hConnection  : DWORD;
    nNamespace   : UINT;
    fbPowerX     : SIM_MC_Power;
    fbPowerY     : SIM_MC_Power;
    fbMoveX      : SIM_MC_MoveAbsolute;
    fbGearIn     : SIM_MC_GearIn;
    fbGearOut    : SIM_MC_GearOut;
    fbCamIn      : SIM_MC_CamIn;
    fbCamOut     : SIM_MC_CamOut;
    fbStopY      : SIM_MC_Stop;
    tonGeared    : TON;
    nStep        : INT;
    sMaster      : STRING(31) := 'x';    (* axis ids of the simulated machine: a two-axis gantry *)
    sSlave       : STRING(31) := 'y';
END_VAR

CASE nStep OF
0:  stSession.sApplicationName := 'TwinCAT sim client';
    stSession.eSecurityMode := eUASecurityMsgMode_None;
    stSession.eSecurityPolicyUri := eUASecurityPolicy_None;
    stSession.eTransportProfileUri := eUATransportProfileUri_UATcp;
    stSession.tSessionTimeout := T#60S;
    fbConnect(Execute := TRUE, ServerURL := 'opc.tcp://192.168.0.10:4840', SessionConnectInfo := stSession, Timeout := T#5S);
    IF fbConnect.Done THEN hConnection := fbConnect.ConnectionHdl; fbConnect(Execute := FALSE); nStep := 10;
    ELSIF fbConnect.Error THEN fbConnect(Execute := FALSE); nStep := 900; END_IF
10: fbNamespace(Execute := TRUE, ConnectionHdl := hConnection, NamespaceUri := 'urn:kinematics-simulation', Timeout := T#5S);
    IF fbNamespace.Done THEN nNamespace := fbNamespace.NamespaceIndex; fbNamespace(Execute := FALSE); nStep := 20;
    ELSIF fbNamespace.Error THEN fbNamespace(Execute := FALSE); nStep := 900; END_IF
20: fbPowerX.Enable := TRUE; fbPowerY.Enable := TRUE;
    IF fbPowerX.Status AND fbPowerY.Status THEN nStep := 30; END_IF
30: (* the master runs slowly over its stroke *)
    fbMoveX.Position := 0.9; fbMoveX.Velocity := 0.05; fbMoveX.Acceleration := 1.0; fbMoveX.Deceleration := 1.0;
    fbMoveX.Jerk := 100.0; fbMoveX.Execute := TRUE;
    IF fbMoveX.Busy THEN nStep := 40; ELSIF fbMoveX.Error THEN nStep := 900; END_IF
40: (* y gears in at 2:1 *)
    fbGearIn.RatioNumerator := 2.0; fbGearIn.RatioDenominator := 1.0;
    fbGearIn.Acceleration := 1.0; fbGearIn.Deceleration := 1.0; fbGearIn.Jerk := 100.0;
    fbGearIn.Execute := TRUE;
    tonGeared(IN := fbGearIn.InGear, PT := T#1S);
    IF tonGeared.Q THEN nStep := 50; ELSIF fbGearIn.Error OR fbGearIn.CommandAborted THEN nStep := 900; END_IF
50: fbGearOut.Execute := TRUE;
    IF fbGearOut.Done THEN fbGearOut.Execute := FALSE; fbGearIn.Execute := FALSE; nStep := 60;
    ELSIF fbGearOut.Error THEN nStep := 900; END_IF
60: (* y cams in, relative, on a 3-4-5 rise of 20 mm over 0.1 m of x and back *)
    fbCamIn.sTable := '0.1, 0.02, 3-4-5$N0.1, -0.02, 3-4-5';
    fbCamIn.StartMode := 1.0; fbCamIn.CamTableID := 0.0;
    fbCamIn.Execute := TRUE;
    IF fbCamIn.InSync THEN nStep := 70; ELSIF fbCamIn.Error THEN nStep := 900; END_IF
70: IF NOT fbMoveX.Busy THEN nStep := 80; END_IF   (* x has arrived; y stands on its cam *)
80: fbCamOut.Execute := TRUE;
    IF fbCamOut.Done THEN fbCamOut.Execute := FALSE; fbCamIn.Execute := FALSE; nStep := 90;
    ELSIF fbCamOut.Error THEN nStep := 900; END_IF
90: fbStopY.Deceleration := 1.0; fbStopY.Execute := TRUE;
    IF fbStopY.Done THEN fbStopY.Execute := FALSE; fbMoveX.Execute := FALSE; nStep := 95;
    ELSIF fbStopY.Error THEN nStep := 900; END_IF
95: fbPowerX.Enable := FALSE; fbPowerY.Enable := FALSE;
    IF NOT fbPowerX.Status AND NOT fbPowerY.Status THEN nStep := 99; END_IF
99: fbDisconnect(Execute := TRUE, ConnectionHdl := hConnection, Timeout := T#5S);
    IF fbDisconnect.Done OR fbDisconnect.Error THEN fbDisconnect(Execute := FALSE); nStep := 100; END_IF
100: ; (* finished *)
900: ; (* error: look at the blocks' ErrorID *)
END_CASE

IF nStep >= 20 AND nStep < 99 THEN
    fbPowerX(hConnection := hConnection, nNamespace := nNamespace, sAxis := sMaster);
    fbPowerY(hConnection := hConnection, nNamespace := nNamespace, sAxis := sSlave);
    fbMoveX(hConnection := hConnection, nNamespace := nNamespace, sAxis := sMaster);
    fbGearIn(hConnection := hConnection, nNamespace := nNamespace, sMaster := sMaster, sSlave := sSlave);
    fbGearOut(hConnection := hConnection, nNamespace := nNamespace, sSlave := sSlave);
    fbCamIn(hConnection := hConnection, nNamespace := nNamespace, sMaster := sMaster, sSlave := sSlave);
    fbCamOut(hConnection := hConnection, nNamespace := nNamespace, sSlave := sSlave);
    fbStopY(hConnection := hConnection, nNamespace := nNamespace, sAxis := sSlave);
END_IF
END_PROGRAM

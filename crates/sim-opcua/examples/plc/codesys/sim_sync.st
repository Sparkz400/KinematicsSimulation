(*
    PLCopen synchronised motion over OPC UA to the KinematicsSimulation link (sim-opcua), for CODESYS V3.5:
    MC_GearIn, MC_GearOut, MC_CamIn and MC_CamOut on a slave axis, its master another axis of the machine.
    Needs SIM_Access and SIM_Handshake from sim_plcopen.st and SIM_WriteString from sim_cycle.st in the same project.

    NOT COMPILED, as sim_plcopen.st (see its header for the library and the names). It uses only the blocks of
    sim_plcopen.st and sim_cycle.st and plain IEC 61131-3. SIM_WriteString writes a STRING(255): a cam table
    (CamIn.Table) longer than that needs a longer STRING there.

    The structure is the one of ../twincat/sim_sync.st:
      SIM_SyncHandshake    the handshake of MC_GearIn or MC_CamIn: inputs and texts, Execute low, Execute high, then
                           poll Busy, InGear/InSync, CommandAborted, Error and ErrorID for as long as the block is busy:
                           unlike a move it stays busy while the slave is coupled
      SIM_MC_GearIn, SIM_MC_CamIn, SIM_MC_GearOut, SIM_MC_CamOut   thin wrappers with PLCopen inputs and outputs
      PRG_SimSyncDemo      x moves, y gears in at 2:1, gears out, cams in on a 3-4-5 table, cams out, stops
*)

FUNCTION_BLOCK SIM_SyncHandshake
VAR_INPUT
    Execute     : BOOL;
    hConnection : DWORD;
    uiNamespace : UINT;
    sAxis       : STRING(31);                  (* the slave's axis id *)
    sFb         : STRING(31);                  (* 'GearIn' or 'CamIn' *)
    sMaster     : STRING(31);                  (* the master's axis id; '' for the machine's virtual cam master *)
    sTable      : STRING(255);                 (* CamIn only: the cam table, '' to leave it as it is *)
    iParams     : INT;                         (* 0..6 *)
    asParam     : ARRAY[1..6] OF STRING(31);
    alrParam    : ARRAY[1..6] OF LREAL;
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
    fbText   : SIM_WriteString;
    iPhase   : INT;      (* 0 idle, 1 inputs, 2 Master, 3 Table, 4 Execute low, 5 Execute high, 6 poll, 7 finished *)
    iItem    : INT;
    xLast    : BOOL;
    xStarted : BOOL;
    xValue   : BOOL;
    lrValue  : LREAL;
    uiValue  : UINT;
    sText    : STRING(255);
    axRemote : ARRAY[1..4] OF BOOL;   (* Busy, InGear/InSync, CommandAborted, Error as read *)
    sBase    : STRING(80);
    sSync    : STRING(31);
END_VAR

IF Execute AND NOT xLast THEN
    InSync := FALSE; Busy := TRUE; Active := FALSE; CommandAborted := FALSE; Error := FALSE; ErrorID := 0;
    sBase := CONCAT(CONCAT(CONCAT('axis.', sAxis), '.'), CONCAT(sFb, '.'));
    IF sFb = 'GearIn' THEN sSync := 'InGear'; ELSE sSync := 'InSync'; END_IF
    IF iParams > 0 THEN iPhase := 1; ELSE iPhase := 2; END_IF
    iItem := 1; xStarted := FALSE;
ELSIF NOT Execute AND xLast THEN
    (* the outputs fall with Execute; a coupled block stays coupled until GearOut/CamOut or another block *)
    InSync := FALSE; CommandAborted := FALSE; Error := FALSE; ErrorID := 0;
    iPhase := 4; xStarted := FALSE;
END_IF
xLast := Execute;

IF iPhase = 2 OR iPhase = 3 THEN
    (* the texts: the master, then (CamIn) the table *)
    IF iPhase = 3 AND (sFb <> 'CamIn' OR LEN(sTable) = 0) THEN
        iPhase := 4;
    ELSE
        IF NOT xStarted THEN
            IF iPhase = 2 THEN
                fbText.sNode := CONCAT(sBase, 'Master'); sText := sMaster;
            ELSE
                fbText.sNode := CONCAT(sBase, 'Table'); sText := sTable;
            END_IF
            fbText(xExecute := FALSE, hConnection := hConnection, uiNamespace := uiNamespace, sValue := sText);
            xStarted := TRUE;
        END_IF
        fbText(xExecute := TRUE, hConnection := hConnection, uiNamespace := uiNamespace, sValue := sText);
        IF fbText.xError THEN
            Error := TRUE; ErrorID := 16#FFFF; Busy := FALSE; iPhase := 7;
        ELSIF fbText.xDone THEN
            xStarted := FALSE; iPhase := iPhase + 1;
        END_IF
    END_IF
ELSIF iPhase > 0 AND iPhase < 7 THEN
    IF NOT xStarted THEN
        CASE iPhase OF
        1:  fbAccess.sNode := CONCAT(sBase, asParam[iItem]); lrValue := alrParam[iItem];
            fbAccess.eKind := SIM_LREAL; fbAccess.xWrite := TRUE;
        4, 5:
            fbAccess.sNode := CONCAT(sBase, 'Execute'); xValue := iPhase = 5 AND Execute;
            fbAccess.eKind := SIM_BOOL; fbAccess.xWrite := TRUE;
        6:  CASE iItem OF
            1: fbAccess.sNode := CONCAT(sBase, 'Busy');
            2: fbAccess.sNode := CONCAT(sBase, sSync);
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
        Error := TRUE; ErrorID := 16#FFFF; Busy := FALSE; iPhase := 7;   (* the OPC UA access failed *)
    ELSIF fbAccess.xDone THEN
        xStarted := FALSE;
        CASE iPhase OF
        1:  iItem := iItem + 1;
            IF iItem > iParams THEN iPhase := 2; iItem := 1; END_IF
        4:  IF Execute THEN iPhase := 5; ELSE iPhase := 0; Busy := FALSE; Active := FALSE; END_IF
        5:  iPhase := 6; iItem := 1;
        6:  IF iItem <= 4 THEN
                axRemote[iItem] := xValue;
                iItem := iItem + 1;
            ELSE
                ErrorID := uiValue;
                Busy := axRemote[1]; Active := axRemote[1];
                InSync := axRemote[2] AND Execute;
                CommandAborted := axRemote[3] AND Execute;
                Error := axRemote[4] AND Execute;
                (* coupled, the block stays busy: poll on; it is over once it is no longer busy *)
                IF NOT Busy THEN iPhase := 7; END_IF
                iItem := 1;
            END_IF
        END_CASE
    END_IF
END_IF
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_MC_GearIn
VAR_INPUT
    Execute : BOOL; hConnection : DWORD; uiNamespace : UINT;
    sMaster : STRING(31); sSlave : STRING(31);
    RatioNumerator : LREAL; RatioDenominator : LREAL; Acceleration : LREAL; Deceleration : LREAL; Jerk : LREAL;
END_VAR
VAR_OUTPUT
    InGear : BOOL; Busy : BOOL; Active : BOOL; CommandAborted : BOOL; Error : BOOL; ErrorID : UINT;
END_VAR
VAR
    fbCall : SIM_SyncHandshake;
END_VAR
fbCall.asParam[1] := 'RatioNumerator';   fbCall.alrParam[1] := RatioNumerator;
fbCall.asParam[2] := 'RatioDenominator'; fbCall.alrParam[2] := RatioDenominator;
fbCall.asParam[3] := 'Acceleration';     fbCall.alrParam[3] := Acceleration;
fbCall.asParam[4] := 'Deceleration';     fbCall.alrParam[4] := Deceleration;
fbCall.asParam[5] := 'Jerk';             fbCall.alrParam[5] := Jerk;
fbCall(Execute := Execute, hConnection := hConnection, uiNamespace := uiNamespace, sAxis := sSlave, sFb := 'GearIn',
       sMaster := sMaster, sTable := '', iParams := 5);
InGear := fbCall.InSync; Busy := fbCall.Busy; Active := fbCall.Active;
CommandAborted := fbCall.CommandAborted; Error := fbCall.Error; ErrorID := fbCall.ErrorID;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_MC_CamIn
VAR_INPUT
    Execute : BOOL; hConnection : DWORD; uiNamespace : UINT;
    sMaster : STRING(31); sSlave : STRING(31);
    MasterOffset : LREAL; SlaveOffset : LREAL; MasterScaling : LREAL := 1.0; SlaveScaling : LREAL := 1.0;
    StartMode : LREAL := 1.0;    (* 0 absolute, 1 relative *)
    CamTableID : LREAL;          (* 0 the table below, 1 the machine's own cam *)
    sTable : STRING(255);        (* lines 'master span, slave rise, law' or 'master, slave[, slope]' *)
END_VAR
VAR_OUTPUT
    InSync : BOOL; Busy : BOOL; Active : BOOL; CommandAborted : BOOL; Error : BOOL; ErrorID : UINT;
END_VAR
VAR
    fbCall : SIM_SyncHandshake;
END_VAR
fbCall.asParam[1] := 'MasterOffset';  fbCall.alrParam[1] := MasterOffset;
fbCall.asParam[2] := 'SlaveOffset';   fbCall.alrParam[2] := SlaveOffset;
fbCall.asParam[3] := 'MasterScaling'; fbCall.alrParam[3] := MasterScaling;
fbCall.asParam[4] := 'SlaveScaling';  fbCall.alrParam[4] := SlaveScaling;
fbCall.asParam[5] := 'StartMode';     fbCall.alrParam[5] := StartMode;
fbCall.asParam[6] := 'CamTableID';    fbCall.alrParam[6] := CamTableID;
fbCall(Execute := Execute, hConnection := hConnection, uiNamespace := uiNamespace, sAxis := sSlave, sFb := 'CamIn',
       sMaster := sMaster, sTable := sTable, iParams := 6);
InSync := fbCall.InSync; Busy := fbCall.Busy; Active := fbCall.Active;
CommandAborted := fbCall.CommandAborted; Error := fbCall.Error; ErrorID := fbCall.ErrorID;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_MC_GearOut
VAR_INPUT
    Execute : BOOL; hConnection : DWORD; uiNamespace : UINT; sSlave : STRING(31);
END_VAR
VAR_OUTPUT
    Done : BOOL; Busy : BOOL; Error : BOOL; ErrorID : UINT;
END_VAR
VAR
    fbCall : SIM_Handshake;
END_VAR
fbCall(Execute := Execute, hConnection := hConnection, uiNamespace := uiNamespace, sAxis := sSlave, sFb := 'GearOut', iParams := 0);
Done := fbCall.Done; Busy := fbCall.Busy; Error := fbCall.Error; ErrorID := fbCall.ErrorID;
END_FUNCTION_BLOCK

FUNCTION_BLOCK SIM_MC_CamOut
VAR_INPUT
    Execute : BOOL; hConnection : DWORD; uiNamespace : UINT; sSlave : STRING(31);
END_VAR
VAR_OUTPUT
    Done : BOOL; Busy : BOOL; Error : BOOL; ErrorID : UINT;
END_VAR
VAR
    fbCall : SIM_Handshake;
END_VAR
fbCall(Execute := Execute, hConnection := hConnection, uiNamespace := uiNamespace, sAxis := sSlave, sFb := 'CamOut', iParams := 0);
Done := fbCall.Done; Busy := fbCall.Busy; Error := fbCall.Error; ErrorID := fbCall.ErrorID;
END_FUNCTION_BLOCK

PROGRAM PRG_SimSyncDemo
VAR
    fbConnect    : UA_Connect;
    fbNamespace  : UA_NamespaceGetIndex;
    fbDisconnect : UA_Disconnect;
    stSession    : UASessionConnectInfo;
    hConnection  : DWORD;
    uiNamespace  : UINT;
    fbPowerX     : SIM_MC_Power;
    fbPowerY     : SIM_MC_Power;
    fbMoveX      : SIM_MC_MoveAbsolute;
    fbGearIn     : SIM_MC_GearIn;
    fbGearOut    : SIM_MC_GearOut;
    fbCamIn      : SIM_MC_CamIn;
    fbCamOut     : SIM_MC_CamOut;
    fbStopY      : SIM_MC_Stop;
    tonGeared    : TON;
    iStep        : INT;
    sMaster      : STRING(31) := 'x';    (* axis ids of the simulated machine: a two-axis gantry *)
    sSlave       : STRING(31) := 'y';
END_VAR

CASE iStep OF
0:  stSession.ApplicationName := 'CODESYS sim client';
    stSession.SecurityMsgMode := UASecurityMsgMode.UASMM_None;
    stSession.SecurityPolicy := UASecurityPolicy.UASP_None;
    fbConnect(Execute := TRUE, ServerEndpointUrl := 'opc.tcp://192.168.0.10:4840', SessionConnectInfo := stSession, Timeout := T#5S);
    IF fbConnect.Done THEN hConnection := fbConnect.ConnectionHdl; fbConnect(Execute := FALSE); iStep := 10;
    ELSIF fbConnect.Error THEN fbConnect(Execute := FALSE); iStep := 900; END_IF
10: fbNamespace(Execute := TRUE, ConnectionHdl := hConnection, NamespaceUri := 'urn:kinematics-simulation', Timeout := T#5S);
    IF fbNamespace.Done THEN uiNamespace := fbNamespace.NamespaceIndex; fbNamespace(Execute := FALSE); iStep := 20;
    ELSIF fbNamespace.Error THEN fbNamespace(Execute := FALSE); iStep := 900; END_IF
20: fbPowerX.Enable := TRUE; fbPowerY.Enable := TRUE;
    IF fbPowerX.Status AND fbPowerY.Status THEN iStep := 30; END_IF
30: (* the master runs slowly over its stroke *)
    fbMoveX.Position := 0.9; fbMoveX.Velocity := 0.05; fbMoveX.Acceleration := 1.0; fbMoveX.Deceleration := 1.0;
    fbMoveX.Jerk := 100.0; fbMoveX.Execute := TRUE;
    IF fbMoveX.Busy THEN iStep := 40; ELSIF fbMoveX.Error THEN iStep := 900; END_IF
40: (* y gears in at 2:1 *)
    fbGearIn.RatioNumerator := 2.0; fbGearIn.RatioDenominator := 1.0;
    fbGearIn.Acceleration := 1.0; fbGearIn.Deceleration := 1.0; fbGearIn.Jerk := 100.0;
    fbGearIn.Execute := TRUE;
    tonGeared(IN := fbGearIn.InGear, PT := T#1S);
    IF tonGeared.Q THEN iStep := 50; ELSIF fbGearIn.Error OR fbGearIn.CommandAborted THEN iStep := 900; END_IF
50: fbGearOut.Execute := TRUE;
    IF fbGearOut.Done THEN fbGearOut.Execute := FALSE; fbGearIn.Execute := FALSE; iStep := 60;
    ELSIF fbGearOut.Error THEN iStep := 900; END_IF
60: (* y cams in, relative, on a 3-4-5 rise of 20 mm over 0.1 m of x and back *)
    fbCamIn.sTable := '0.1, 0.02, 3-4-5$N0.1, -0.02, 3-4-5';
    fbCamIn.StartMode := 1.0; fbCamIn.CamTableID := 0.0;
    fbCamIn.Execute := TRUE;
    IF fbCamIn.InSync THEN iStep := 70; ELSIF fbCamIn.Error THEN iStep := 900; END_IF
70: IF NOT fbMoveX.Busy THEN iStep := 80; END_IF   (* x has arrived; y stands on its cam *)
80: fbCamOut.Execute := TRUE;
    IF fbCamOut.Done THEN fbCamOut.Execute := FALSE; fbCamIn.Execute := FALSE; iStep := 90;
    ELSIF fbCamOut.Error THEN iStep := 900; END_IF
90: fbStopY.Deceleration := 1.0; fbStopY.Execute := TRUE;
    IF fbStopY.Done THEN fbStopY.Execute := FALSE; fbMoveX.Execute := FALSE; iStep := 95;
    ELSIF fbStopY.Error THEN iStep := 900; END_IF
95: fbPowerX.Enable := FALSE; fbPowerY.Enable := FALSE;
    IF NOT fbPowerX.Status AND NOT fbPowerY.Status THEN iStep := 99; END_IF
99: fbDisconnect(Execute := TRUE, ConnectionHdl := hConnection, Timeout := T#5S);
    IF fbDisconnect.Done OR fbDisconnect.Error THEN fbDisconnect(Execute := FALSE); iStep := 100; END_IF
100: ;
900: ;
END_CASE

IF iStep >= 20 AND iStep < 99 THEN
    fbPowerX(hConnection := hConnection, uiNamespace := uiNamespace, sAxis := sMaster);
    fbPowerY(hConnection := hConnection, uiNamespace := uiNamespace, sAxis := sSlave);
    fbMoveX(hConnection := hConnection, uiNamespace := uiNamespace, sAxis := sMaster);
    fbGearIn(hConnection := hConnection, uiNamespace := uiNamespace, sMaster := sMaster, sSlave := sSlave);
    fbGearOut(hConnection := hConnection, uiNamespace := uiNamespace, sSlave := sSlave);
    fbCamIn(hConnection := hConnection, uiNamespace := uiNamespace, sMaster := sMaster, sSlave := sSlave);
    fbCamOut(hConnection := hConnection, uiNamespace := uiNamespace, sSlave := sSlave);
    fbStopY(hConnection := hConnection, uiNamespace := uiNamespace, sAxis := sSlave);
END_IF
END_PROGRAM

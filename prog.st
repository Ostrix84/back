TYPE
  REAL2 : STRUCT
    R1 : REAL;
    RX : REAL;
  END_STRUCT;
END_TYPE

FUNCTION T_PLC_US : UDINT
  VAR
    tx : UDINT;
  END_VAR
  VAR_INPUT
    debug : BOOL;
  END_VAR
  VAR
    N : INT := 0;
    offset : UDINT := 0;
    temp : DWORD := 1;
  END_VAR

  {extern unsigned long __tick;
  extern unsigned long long common_ticktime__;
  unsigned long long ticktime_ms = (common_ticktime__)/1000000;
  UDINT plc_time = (UDINT)(ticktime_ms * (unsigned long long)__tick);
  TX = plc_time}

  T_PLC_US := tx*1000;
  IF debug THEN
  	T_PLC_US := (DWORD_TO_UDINT(SHL(UDINT_TO_DWORD(T_PLC_US),N) OR SHL(temp,N))-1) + OFFSET;
  END_IF;

  (* Original Code:
  tx := TIME();
  T_PLC_US := TIME_TO_DWORD(Tx)*1000;
  IF debug THEN
  	T_PLC_US := (SHL(T_PLC_US,N) OR SHL(DWORD#1,N)-1) + OFFSET;
  END_IF;
  *)

  (* From OSCAT library, www.oscat.de

  this is a temporary T_PLC_US FB until OpenPLC gets it´s own time() functionality *)

  (* PLC_TIME and Global variables PLC_SCAN_CYCL and PLC_CYCL_TIME required *)
END_FUNCTION

FUNCTION_BLOCK FT_PIWL
  VAR_INPUT
    IN : REAL;
    KP : REAL := 1.0;
    KI : REAL := 1.0;
    LIM_L : REAL := -1.0E38;
    LIM_H : REAL := 1.0E38;
    RST : BOOL;
  END_VAR
  VAR_OUTPUT
    Y : REAL;
    LIM : BOOL;
  END_VAR
  VAR
    init : BOOL;
    tx : UDINT;
    tc : REAL;
    t_last : UDINT;
    in_last : REAL;
    i : REAL;
    p : REAL;
  END_VAR

  IF NOT init OR RST THEN
  	init := TRUE;
  	in_last := in;
  		t_last := T_PLC_US(en:=true);
  	i := 0.0;
  	tc := 0.0;
  ELSE
  	(* read last cycle time in Microseconds *)
  		tx := T_PLC_US(en:=true);
  	tc := UDINT_TO_REAL(tx - t_last);
  	t_last := tx;

  	(* calculate proportional part *)
  	p := KP * IN;

  	(* run integrator *)
  	i := (IN + in_last) * 5.0E-7 * KI * tc + i;
  	in_last := IN;

  	(* calculate output Y *)
  	Y := p + i;

  	(* check output for limits *)
  	IF Y >= LIM_H THEN
  		Y := LIM_H;
  		IF ki <> 0.0 THEN
  			i := LIM_H - p;
  		ELSE
  			i := 0.0;
  		END_IF;
  		LIM := TRUE;
  	ELSIF Y <= LIM_L THEN
  		Y := LIM_L;
  		IF ki <> 0.0 THEN
  			i := LIM_L - p;
  		ELSE
  			i := 0.0;
  		END_IF;
  		LIM := TRUE;
  	ELSE
  		LIM := FALSE;
  	END_IF;
  END_IF;

  (* From OSCAT Library, www.oscat.de *)
  (* T_PLC_US required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK FT_DERIV
  VAR_INPUT
    IN : REAL;
    K : REAL := 1.0;
    RUN : BOOL := TRUE;
  END_VAR
  VAR_OUTPUT
    OUT : REAL;
  END_VAR
  VAR
    old : REAL;
    tx : UDINT;
    last : UDINT;
    init : BOOL;
    tc : REAL;
  END_VAR

  tx:= T_PLC_US(en:=true);
  tc := UDINT_TO_REAL(tx - last);

  (* init on firsat startup *)
  IF NOT init THEN
  	init := TRUE;
  	old := in;
  ELSIF run AND tc > 0.0 THEN
  	out := (in - old) / tc * 1000000.0 * K;
  	old := in;
  ELSE
  	out := 0.0;
  END_IF;

  last := tx;

  (* From OSCAT Library, www.oscat.de *)
  (* T_PLC_US, required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK FT_PIDWL
  VAR_INPUT
    IN : REAL;
    KP : REAL := 1.0;
    TN : REAL := 1.0;
    TV : REAL := 1.0;
    LIM_L : REAL := -1.0E38;
    LIM_H : REAL := 1.0E38;
    RST : BOOL;
  END_VAR
  VAR_OUTPUT
    Y : REAL;
    LIM : BOOL;
  END_VAR
  VAR
    piwl : FT_PIWL;
    diff : FT_DERIV;
  END_VAR

  IF rst THEN
  	piwl(rst := TRUE);
  	piwl.RST := FALSE;
  ELSE
  	(* run PIWL controller first *)
  	(* we need to check if TN = 0 and do alternative calls *)
  	IF TN = 0.0 THEN
  		piwl(in := IN * KP, KP := 1.0, KI := 0.0, LIM_L := LIM_L, LIM_H := LIM_H);
  	ELSE
  		piwl(in := IN * KP, KP := 1.0, KI := 1.0 / TN, LIM_L := LIM_L, LIM_H := LIM_H);
  	END_IF;

  	(* run differentiator and add_to_output *)
  	diff(IN := IN, K := KP * TV);
  	Y := piwl.Y + diff.out;

  	(* limit the output *)
  	IF Y < LIM_L THEN
  		LIM := TRUE;
  		Y := LIM_L;
  	ELSIF Y > LIM_H THEN
  		LIM := TRUE;
  		Y := LIM_H;
  	ELSE
  		LIM := FALSE;
  	END_IF;
  END_IF;



  (* From OSCAT Library, www.oscat.de *)
  (* T_PLC_US, FT_DERIV required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK CTRL_OUT
  VAR_INPUT
    CI : REAL;
    OFFSET : REAL;
    MAN_IN : REAL;
    LIM_L : REAL;
    LIM_H : REAL;
    MANUAL : BOOL;
  END_VAR
  VAR_OUTPUT
    Y : REAL;
    LIM : BOOL;
  END_VAR

  Y := SEL(manual, CI, MAN_IN) + OFFSET;

  (* Limit the output *)
  IF Y >= LIM_H THEN
  	Y := LIM_H;
  	LIM := TRUE;
  ELSIF Y <= lim_L THEN
  	Y := LIM_L;
  	LIM := TRUE;
  ELSE
  	LIM := FALSE;
  END_IF;

  (* From OSCAT Library, www.oscat.de *)
END_FUNCTION_BLOCK

FUNCTION DEAD_ZONE : REAL
  VAR_INPUT
    X : REAL;
    L : REAL;
  END_VAR

  IF ABS(x) > L THEN
  	dead_zone := X;
  ELSE
  	DEAD_ZONE := 0.0;
  END_IF;

  (* From OSCAT Library, www.oscat.de *)
END_FUNCTION

FUNCTION CTRL_IN : REAL
  VAR_INPUT
    SET_POINT : REAL;
    ACTUAL : REAL;
    NOISE : REAL;
  END_VAR

  CTRL_IN := DEAD_ZONE(SET_POINT - ACTUAL, NOISE);

  (* From OSCAT Library, www.oscat.de *)
  (* DEAD_ZONE required *)
END_FUNCTION

FUNCTION_BLOCK CTRL_PID
  VAR_INPUT
    ACT : REAL;
    SET : REAL;
    SUP : REAL;
    OFS : REAL;
    M_I : REAL;
    MAN : BOOL;
    RST : BOOL := FALSE;
    KP : REAL := 1.0;
    TN : REAL := 1.0;
    TV : REAL := 1.0;
    LL : REAL := -1000.0;
    LH : REAL := 1000.0;
  END_VAR
  VAR_OUTPUT
    Y : REAL;
    DIFF : REAL;
    LIM : BOOL;
  END_VAR
  VAR
    _pid : FT_PIDWL;
    co : CTRL_OUT;
  END_VAR

  DIFF := CTRL_IN(SET, ACT, SUP);
  _pid(in := DIFF, kp := KP, tn := TN, tv := TV, lim_l := LL, lim_h := LH, rst := RST);
  co(ci := _pid.Y, OFFSET := OFS, man_in := M_I, lim_l := LL, lim_h := LH, manual := MAN);
  Y := co.Y;
  LIM := co.LIM;


  (* From OSCAT Library, www.oscat.de *)
  (* CTRL_IN, FT_PIDWL, CTRL_out reauired *)
END_FUNCTION_BLOCK

PROGRAM program0
  VAR
    Asked_Temperature AT %IW100 : INT := 30;
    Final_Temperature AT %IW101 : INT := 2599;
  END_VAR
  VAR
    Final_Temperature_1 : INT;
  END_VAR
  VAR
    Power_Fan AT %IW104 : INT := 0;
    Pressure_Difference AT %IW105 : INT := 0;
    Power_Fan_sent AT %QW101 : INT := 0;
    Voltage_sent AT %QW100 : INT;
  END_VAR
  VAR
    Voltage_12V : REAL := 12.0;
    Voltage_Limit : BOOL;
    _100 : REAL := 100.0;
  END_VAR
  VAR
    OnOff AT %IW102 : INT := 0;
    OnOff_Fan AT %IW103 : INT := 0;
  END_VAR
  VAR
    P : REAL := 25.0;
    I : REAL := 10.0;
    _D : REAL := 1.0;
    _0 : REAL := 0.0;
    Test : REAL;
    Test0 : REAL;
    CTRL_PID0 : CTRL_PID;
    Variable : BOOL := 0;
    _TMP_INT_TO_REAL46_OUT : REAL;
    _TMP_DIV14_OUT : REAL;
    _TMP_INT_TO_REAL1_OUT : REAL;
    _TMP_INT_TO_BOOL3_OUT : BOOL;
    _TMP_NOT84_OUT : BOOL;
    _TMP_INT_TO_BOOL76_OUT : BOOL;
    _TMP_AND83_OUT : BOOL;
    _TMP_NOT81_OUT : BOOL;
    _TMP_MUL4_OUT : REAL;
    _TMP_REAL_TO_INT59_OUT : INT;
    _TMP_REAL_TO_INT6_OUT : INT;
    _TMP_GE91_OUT : BOOL;
    _TMP_BOOL_TO_INT92_OUT : INT;
    _TMP_MUL93_OUT : INT;
    _TMP_REAL_TO_INT88_OUT : INT;
    _TMP_LE86_OUT : BOOL;
    _TMP_BOOL_TO_INT9_OUT : INT;
    _TMP_MUL8_OUT : INT;
  END_VAR

  _TMP_INT_TO_REAL46_OUT := INT_TO_REAL(Final_Temperature);
  _TMP_DIV14_OUT := DIV(_TMP_INT_TO_REAL46_OUT, _100);
  _TMP_INT_TO_REAL1_OUT := INT_TO_REAL(Asked_Temperature);
  _TMP_INT_TO_BOOL3_OUT := INT_TO_BOOL(OnOff_Fan);
  _TMP_NOT84_OUT := NOT(_TMP_INT_TO_BOOL3_OUT);
  _TMP_INT_TO_BOOL76_OUT := INT_TO_BOOL(OnOff);
  _TMP_AND83_OUT := AND(_TMP_NOT84_OUT, _TMP_INT_TO_BOOL76_OUT);
  _TMP_NOT81_OUT := NOT(_TMP_AND83_OUT);
  CTRL_PID0(ACT := _TMP_DIV14_OUT, SET := _TMP_INT_TO_REAL1_OUT, M_I := _0, MAN := _TMP_NOT81_OUT, KP := P, TN := I, TV := _D, LL := _0, LH := Voltage_12V);
  _TMP_MUL4_OUT := MUL(CTRL_PID0.Y, _100);
  _TMP_REAL_TO_INT59_OUT := REAL_TO_INT(_TMP_MUL4_OUT);
  Voltage_sent := _TMP_REAL_TO_INT59_OUT;
  Test := CTRL_PID0.Y;
  Voltage_Limit := CTRL_PID0.LIM;
  Test0 := CTRL_PID0.DIFF;
  _TMP_REAL_TO_INT6_OUT := REAL_TO_INT(_0);
  _TMP_GE91_OUT := GE(Power_Fan, _TMP_REAL_TO_INT6_OUT);
  _TMP_BOOL_TO_INT92_OUT := BOOL_TO_INT(_TMP_GE91_OUT);
  _TMP_MUL93_OUT := MUL(_TMP_BOOL_TO_INT92_OUT, Power_Fan);
  _TMP_REAL_TO_INT88_OUT := REAL_TO_INT(_100);
  _TMP_LE86_OUT := LE(_TMP_MUL93_OUT, _TMP_REAL_TO_INT88_OUT);
  _TMP_BOOL_TO_INT9_OUT := BOOL_TO_INT(_TMP_LE86_OUT);
  _TMP_MUL8_OUT := MUL(_TMP_BOOL_TO_INT9_OUT, Power_Fan);
  Power_Fan_sent := _TMP_MUL8_OUT;
END_PROGRAM

FUNCTION T_PLC_MS : UDINT
  VAR
    tx : UDINT;
  END_VAR
  VAR_INPUT
    debug : BOOL;
  END_VAR
  VAR
    N : INT := 0;
    offset : UDINT := 0;
    temp : DWORD := 1;
  END_VAR

  tx := 0;

  {extern unsigned long __tick;
  extern unsigned long long common_ticktime__;
  unsigned long long ticktime_ms = (common_ticktime__)/1000000;
  UDINT plc_time = (UDINT)(ticktime_ms * (unsigned long long)__tick);
  TX = plc_time}

  T_PLC_MS := tx;
  IF debug THEN
  	T_PLC_MS := (DWORD_TO_UDINT(SHL(UDINT_TO_DWORD(T_PLC_MS),N) OR SHL(temp,N))-1) + OFFSET;
  END_IF;

  (* Original Code:
  tx := TIME();
  T_PLC_MS := TIME_TO_DWORD(Tx);
  IF debug THEN
  	T_PLC_MS := (SHL(T_PLC_MS,N) OR SHL(DWORD#1,N)-1) + OFFSET;
  END_IF;
  *)

  (* From OSCAT library, www.oscat.de

  this is a temporary T_PLC_MS FB until OpenPLC gets it´s own time() functionality *)

  (* PLC_TIME and Global variables PLC_SCAN_CYCL and PLC_CYCL_TIME required *)
END_FUNCTION

FUNCTION_BLOCK FT_TN8
  VAR_INPUT
    IN : REAL;
    _T : TIME;
  END_VAR
  VAR_OUTPUT
    OUT : REAL;
    TRIG : BOOL;
  END_VAR
  VAR
    length : INT := 8;
    X : ARRAY [0..7] OF REAL;
    cnt : INT;
    last : TIME;
    tx : TIME;
    init : BOOL;
  END_VAR

  tx:= UDINT_TO_TIME(T_PLC_MS(en:=true));

  trig := FALSE;
  IF NOT init THEN
  	x[cnt] := in;
  	init := TRUE;
  	last := tx;
  ELSIF tx - last >= _T / length THEN
  	IF cnt = length - 1 THEN cnt := 0; ELSE cnt := cnt + 1; END_IF;
  	Out := X[cnt];
  	x[cnt] := in;
  	last := tx;
  	trig := TRUE;
  END_IF;



  (* From OSCAT Library, www.oscat.de *)
  (* T_PLC_MS required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK FT_TN64
  VAR_INPUT
    IN : REAL;
    _T : TIME;
  END_VAR
  VAR_OUTPUT
    OUT : REAL;
    TRIG : BOOL;
  END_VAR
  VAR
    length : INT := 64;
    X : ARRAY [0..63] OF REAL;
    cnt : INT;
    last : TIME;
    tx : TIME;
    init : BOOL;
  END_VAR

  tx:= UDINT_TO_TIME(T_PLC_MS(en:=true));

  trig := FALSE;
  IF NOT init THEN
  	x[cnt] := in;
  	init := TRUE;
  	last := tx;
  ELSIF tx - last >= _T / length THEN
  	IF cnt = length - 1 THEN cnt := 0; ELSE cnt := cnt + 1; END_IF;
  	Out := X[cnt];
  	x[cnt] := in;
  	last := tx;
  	trig := TRUE;
  END_IF;


  (* From OSCAT Library, www.oscat.de *)
  (* T_PLC_MS required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK INTEGRATE
  VAR_INPUT
    _E : BOOL := TRUE;
    X : REAL;
    K : REAL := 1.0;
  END_VAR
  VAR_IN_OUT
    Y : REAL;
  END_VAR
  VAR
    x_last : REAL;
    init : BOOL;
    last : UDINT;
    tx : UDINT;
  END_VAR

  tx:= T_PLC_MS(en:=true);

  IF NOT init THEN
  	init := TRUE;
  	X_last := X;
  ELSIF _E THEN
  	Y := (X + X_LAST) * 0.5E-3 * UDINT_TO_REAL(tx-last) * K + Y;
  	X_last := X;
  END_IF;
  last := tx;

  (* From OSCAT Library, www.oscat.de *)
  (* T_PLC_MS required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK FT_INT
  VAR_INPUT
    IN : REAL;
    K : REAL := 1.0;
    RUN : BOOL := TRUE;
    RST : BOOL;
    OUT_MIN : REAL := -1.0E-37;
    OUT_MAX : REAL := 1.0E-37;
  END_VAR
  VAR_OUTPUT
    OUT : REAL;
    LIM : BOOL;
  END_VAR
  VAR
    INTEG : INTEGRATE;
  END_VAR

  IF rst THEN
  	out := 0.0;
  ELSE
  	integ(X := IN, _E := RUN, K := K, Y := out);
  	out := integ.Y;
  END_IF;

  (* limit the outputs *)
  IF out >= OUT_MAX THEN
  	out := out_max;
  	LIM := TRUE;
  ELSIF out <= out_min THEN
  	out := out_min;
  	lim := TRUE;
  ELSE
  	lim := FALSE;
  END_IF;

  (* From OSCAT Library, www.oscat.de *)
  (* INTEGRATE required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK FT_PIW
  VAR_INPUT
    IN : REAL;
    KP : REAL := 1.0;
    KI : REAL := 1.0;
    LIM_L : REAL := -1.0e38;
    LIM_H : REAL := 1.0e38;
    RST : BOOL := FALSE;
  END_VAR
  VAR_OUTPUT
    Y : REAL;
    LIM : BOOL;
  END_VAR
  VAR
    integ : FT_INT;
  END_VAR

  (* run integrator *)
  integ(IN := IN, K := KI, RUN := NOT LIM, RST := RST);

  (* set output value *)
  Y := KP * IN + integ.Out;

  (* check for limits *)
  IF Y < LIM_L THEN
  	Y := LIM_L;
  	LIM := TRUE;
  ELSIF Y > LIM_H THEN
  	Y := LIM_H;
  	LIM := TRUE;
  ELSE
  	LIM := FALSE;
  END_IF;


  (* From OSCAT Library, www.oscat.de *)
  (* FT_INT required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK FT_PIDW
  VAR_INPUT
    IN : REAL;
    KP : REAL := 1.0;
    TN : REAL := 1.0;
    TV : REAL := 1.0;
    LIM_L : REAL := -1.0e38;
    LIM_H : REAL := 1.0e38;
    RST : BOOL := FALSE;
  END_VAR
  VAR_OUTPUT
    Y : REAL;
    LIM : BOOL;
  END_VAR
  VAR
    integ : INTEGRATE;
    diff : FT_DERIV;
    YI : REAL;
  END_VAR

  (* run the integrator *)
  IF tn = 0.0 OR rst THEN
  	integ(_E := FALSE, Y := YI);
  	YI := integ.Y;
  	YI := 0.0;
  ELSE
  	integ(X := IN, K := 1.0 / TN, _E := NOT LIM, Y := YI);
  	YI := integ.Y;
  END_IF;

  (* add up integrator and linear part *)
  Y := KP * (IN + YI);

  (* run differentiator *)
  diff(IN := IN, K := TV);

  (* set lim before differentiator is added to stop integrator if necessary *)
  IF Y > LIM_L AND Y < LIM_H THEN
  	LIM := FALSE;
  ELSE
  	LIM := TRUE;
  END_IF;

  (* add differential part and limit output Y *)

  Y := LIMIT(LIM_L, Y + KP * diff.out, LIM_H);



  (* From OSCAT Library, www.oscat.de *)
  (* FT_INT,FT_DERIV required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK FT_PID
  VAR_INPUT
    IN : REAL;
    KP : REAL := 1.0;
    TN : REAL := 1.0;
    TV : REAL := 1.0;
    ILIM_L : REAL := -1.0e38;
    ILIM_H : REAL := 1.0e38;
    IEN : BOOL := TRUE;
    RST : BOOL := FALSE;
  END_VAR
  VAR_OUTPUT
    Y : REAL;
    LIM : BOOL;
  END_VAR
  VAR
    integ : FT_INT;
    diff : FT_DERIV;
  END_VAR

  (* run integrator only if TN > 0 *)
  IF TN > 0.0 THEN
  	integ(IN := IN, K := 1.0 / TN, RUN := IEN, RST := RST, OUT_MIN := ILIM_L, OUT_MAX := ILIM_H);
  ELSE
  	integ(RST := FALSE);
  END_IF;

  (* run differentiator *)
  diff(IN := IN, K := TV);

  (* combine both values *)
  Y := KP * (integ.Out + diff.out + IN);

  (* check if integrator has reached its limits and set overflow *)
  LIM := integ.LIM;


  (* From OSCAT Library, www.oscat.de *)
  (* FT_INT,FT_DERIV required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK R2_ADD
  VAR_INPUT
    X : REAL2;
    Y : REAL;
    _R2_ADD : REAL2;
  END_VAR
  VAR
    temp : REAL;
  END_VAR

  temp := X.RX;
  _R2_ADD.RX := Y + X.R1 + X.RX;
  _R2_ADD.R1 := temp - _R2_ADD.RX + Y + X.R1;

  (* From OSCAT Library, www.oscat.de *)
  (* Datatype REAL2 required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK R2_SET
  VAR_INPUT
    X : REAL;
  END_VAR
  VAR_OUTPUT
    _R2_SET : REAL2;
  END_VAR

  _R2_SET.RX := X;
  _R2_SET.R1 := 0.0;

  (* From OSCAT Library, www.oscat.de *)
  (* Datatype REAL2 required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK FT_INT2
  VAR_INPUT
    IN : REAL;
    K : REAL := 1.0;
    RUN : BOOL := TRUE;
    RST : BOOL;
    OUT_MIN : REAL := -1.0E-37;
    OUT_MAX : REAL := 1.0E37;
  END_VAR
  VAR_OUTPUT
    OUT : REAL;
    LIM : BOOL;
  END_VAR
  VAR
    INTEG : INTEGRATE;
    ix : REAL;
    val : REAL2;
    _R2_ADD : R2_ADD;
    _R2_SET : R2_SET;
  END_VAR

  IF RST THEN
  	_R2_SET(X:=0.0);
  	val:=_R2_SET._R2_SET;
  	out := 0.0;
  ELSE
  	integ(X := IN, _E := RUN, K := K, Y := ix);
  	ix := integ.Y;
  	_R2_ADD(X:=val,Y:=ix);
  	val:=_R2_ADD._R2_ADD;
  	ix := 0.0;
  	OUT := val.RX;
  END_IF;

  (* check output for limits *)
  IF out > OUT_MIN AND out < OUT_MAX THEN
  	LIM := FALSE;
  ELSE
  	OUT := LIMIT(OUT_MIN, OUT, OUT_MAX);
  	_R2_SET(X:=OUT);
  	val:=_R2_SET._R2_SET;
  	LIM := TRUE;
  END_IF;


  (* From OSCAT Library, www.oscat.de *)
  (* INTEGRATE required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK TP_X
  VAR_INPUT
    IN : BOOL;
    PT : TIME;
  END_VAR
  VAR_OUTPUT
    Q : BOOL;
    ET : TIME;
  END_VAR
  VAR
    edge : BOOL;
    start : TIME;
    tx : TIME;
  END_VAR

  tx := UDINT_TO_TIME(T_PLC_MS(en:=true));

  (* rising edge trigger *)
  IF IN AND NOT edge THEN
  	start := tx;
  	Q := PT > t#0ms;
  ELSIF Q THEN
  	ET := tx - start;
  	IF ET >= PT THEN
  		Q := FALSE;
  		ET := t#0ms;
  	END_IF;
  END_IF;
  edge := IN;

  (*From OSCAT Library, www.oscat.de *)
  (* T_PLC_MS required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK CLK_PRG
  VAR_INPUT
    PT : TIME := T#10ms;
  END_VAR
  VAR_OUTPUT
    Q : BOOL;
  END_VAR
  VAR
    init : BOOL := FALSE;
    last : TIME;
    tx : TIME;
  END_VAR


  tx:= UDINT_TO_TIME(T_PLC_MS(en:=true));

  (* initialize on startup *)
  IF NOT init THEN
  	init := TRUE;
  	last := tx - pt;
  END_IF;

  (* generate output pulse when next_pulse is reached *)
  Q := tx - last >= pt;
  IF Q THEN last := tx; END_IF;

  (* From OSCAT Library, www.oscat.de *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK PWM_DC
  VAR_INPUT
    F : REAL;
    DC : REAL;
  END_VAR
  VAR_OUTPUT
    Q : BOOL;
  END_VAR
  VAR
    clk : CLK_PRG;
    pulse : TP_X;
    tmp : REAL;
  END_VAR

  IF F > 0.0 THEN
  	tmp := 1000.0/F;
  	CLK(PT := REAL_TO_TIME(tmp));
  	Pulse(in := clk.Q, pt := REAL_TO_TIME(tmp*DC));
  	Q := pulse.Q;
  END_IF;

  (* From OSCAT Library, www.oscat.de *)
  (* T_PLC_MS, CLK_PRG, TP_X required*)
END_FUNCTION_BLOCK

FUNCTION_BLOCK CONTROL_SET12
  VAR_INPUT
    KS : REAL;
    TU : REAL;
    TG : REAL;
    _PI : BOOL;
    _PID : BOOL;
    P_K : REAL := 1.0;
    PI_K : REAL := 0.9;
    PI_TN : REAL := 3.33;
    PID_K : REAL := 1.2;
    PID_TN : REAL := 2.0;
    PID_TV : REAL := 0.5;
  END_VAR
  VAR_OUTPUT
    KP : REAL;
    TN : REAL;
    TV : REAL;
    KI : REAL;
    KD : REAL;
  END_VAR
  VAR
    TX : REAL;
  END_VAR

  IF TU > 0.0 AND KS > 0.0 THEN TX := TG / TU / KS; END_IF;
  IF _pi AND _PID THEN
  	KP := 0.0;
  	TN := 0.0;
  	TV := 0.0;
  ELSIF _PID THEN
  	KP := PID_K * TX;
  	TN := PID_TN * TU;
  	TV := PID_TV * TU;
  ELSIF _PI THEN
  	KP := PI_K * TX;
  	TN := PI_TN * TU;
  ELSE
  	KP := P_K * TX;
  END_IF;

  (* KI and KD are calculated *)
  IF TN > 0.0 THEN KI := KP / TN; ELSE KI := 0.0; END_IF;
  KD := KP * TV;


  (* From OSCAT Library, www.oscat.de *)
END_FUNCTION_BLOCK

FUNCTION BAND_B : BYTE
  VAR_INPUT
    X : BYTE;
    B : BYTE;
  END_VAR

  IF X < B THEN
  	BAND_B := BYTE#0;
  ELSIF X > USINT_TO_BYTE(USINT#255 - BYTE_TO_USINT(B)) THEN
  	BAND_B := BYTE#255;
  ELSE
  	BAND_B := X;
  END_IF;

  (* From OSCAT Library, www.oscat.de *)
END_FUNCTION

FUNCTION_BLOCK CONTROL_SET1
  VAR_INPUT
    KT : REAL;
    TT : REAL;
    _PI : BOOL;
    _PID : BOOL;
    P_K : REAL := 0.5;
    PI_K : REAL := 0.45;
    PI_TN : REAL := 0.83;
    PID_K : REAL := 0.6;
    PID_TN : REAL := 0.5;
    PID_TV : REAL := 0.125;
  END_VAR
  VAR_OUTPUT
    KP : REAL;
    TN : REAL;
    TV : REAL;
    KI : REAL;
    KD : REAL;
  END_VAR

  IF _PI AND _PID THEN
  	KP := 0.0;
  	TN := 0.0;
  	TV := 0.0;
  ELSIF _PID THEN
  	KP := PID_K * Kt;
  	TN := PID_TN * Tt;
  	TV := PID_TV * Tt;
  ELSIF _PI THEN
  	KP := PI_K * Kt;
  	TN := PI_TN * Tt;
  ELSE
  	KP := P_K * Kt;
  END_IF;

  (* KI and KD are calculated *)
  IF tn > 0.0 THEN KI := KP / TN; ELSE KI := 0.0; END_IF;
  KD := KP * TV;

  (* From OSCAT Library, www.oscat.de *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK CTRL_PI
  VAR_INPUT
    ACT : REAL;
    SET : REAL;
    SUP : REAL;
    OFS : REAL;
    M_I : REAL;
    MAN : BOOL;
    RST : BOOL := FALSE;
    KP : REAL := 1.0;
    KI : REAL := 1.0;
    LL : REAL := -1000.0;
    LH : REAL := 1000.0;
  END_VAR
  VAR_OUTPUT
    Y : REAL;
    DIFF : REAL;
    LIM : BOOL;
  END_VAR
  VAR
    pi : FT_PIWL;
    co : CTRL_OUT;
  END_VAR

  DIFF := CTRL_IN(SET, ACT, SUP);
  pi(in := DIFF, kp := KP, ki := KI, lim_l := LL, lim_h := LH, rst := RST);
  co(ci := pi.Y, OFFSET := OFS, man_in := M_I, lim_l := LL, lim_h := LH, manual := MAN);
  Y := co.Y;
  LIM := co.LIM;

  (* From OSCAT Library, www.oscat.de *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK CTRL_PWM
  VAR_INPUT
    CI : REAL;
    MAN_IN : REAL;
    MANUAL : BOOL;
    F : REAL;
  END_VAR
  VAR_OUTPUT
    Q : BOOL;
  END_VAR
  VAR
    PW : PWM_DC;
  END_VAR

  PW(F := F, DC := SEL(MANUAL,CI,MAN_IN));
  Q := PW.Q;

  (* From OSCAT Library, www.oscat.de *)
  (* PWM_DC required*)
END_FUNCTION_BLOCK

FUNCTION DEAD_BAND : REAL
  VAR_INPUT
    X : REAL;
    L : REAL;
  END_VAR

  IF X > L THEN
  	DEAD_BAND := X - L;
  ELSIF X < -L THEN
  	DEAD_BAND := X + L;
  ELSE
  	DEAD_BAND := 0.0;
  END_IF;

  (* From OSCAT Library, www.oscat.de *)
END_FUNCTION

FUNCTION_BLOCK FT_PT1
  VAR_INPUT
    IN : REAL;
    _T : TIME;
    K : REAL := 1.0;
    OUT : REAL;
  END_VAR
  VAR
    last : UDINT;
    tx : UDINT;
    init : BOOL;
  END_VAR

  tx:= T_PLC_US(en:=true);

  (* startup initialisation *)
  IF NOT init OR _T = t#0s THEN
  	init := TRUE;
  	out := K * in;
  ELSE
  	out := out + (in * K - out) * UDINT_TO_REAL(Tx - last) / TIME_TO_REAL(_T) * 1.0E-3;
  	IF ABS(out) < 1.0E-20 THEN out := 0.0; END_IF;
  END_IF;
  last := tx;

  (* From OSCAT Library, www.oscat.de *)
  (* T_PLC_US required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK DEAD_BAND_A
  VAR_INPUT
    X : REAL;
    _T : TIME;
    KL : REAL := 1.0;
    LM : REAL;
  END_VAR
  VAR_OUTPUT
    Y : REAL;
    L : REAL;
  END_VAR
  VAR
    tp1 : FT_PT1;
    tp2 : FT_PT1;
  END_VAR

  (* filter the input signal *)
  tp1(in := X, _T:= _T);

  (* filter the HF portion to generate a stable L *)
  tp2(in := ABS(tp1.out - X), _T := MULTIME(_T, 4.0));

  (* now we determine L which is half the width of the dead band *)
  L := MIN(KL * tp2.out, LM);

  IF X > L THEN
  	Y := X - L;
  ELSIF X < -L THEN
  	Y := X + L;
  ELSE
  	Y := 0.0;
  END_IF;
END_FUNCTION_BLOCK

FUNCTION_BLOCK DEAD_ZONE2
  VAR_INPUT
    X : REAL;
    L : REAL;
  END_VAR
  VAR_OUTPUT
    Y : REAL;
  END_VAR

  IF ABS(x) > L THEN
  	Y := X;
  ELSIF Y > 0.0 THEN
  	Y := L;
  ELSE
  	Y := -L;
  END_IF;

  (* From OSCAT Library, www.oscat.de *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK FT_IMP
  VAR_INPUT
    IN : REAL;
    _T : TIME;
    K : REAL := 1.0;
  END_VAR
  VAR_OUTPUT
    OUT : REAL;
  END_VAR
  VAR
    t1 : FT_PT1;
  END_VAR

  T1(in:= in, _T:=_T);
  out := (in - t1.out) * K;

  (* From OSCAT Library, www.oscat.de *)
  (* FT_PT1 required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK FT_PD
  VAR_INPUT
    IN : REAL;
    KP : REAL := 1.0;
    TV : REAL := 1.0;
  END_VAR
  VAR_OUTPUT
    Y : REAL;
  END_VAR
  VAR
    diff : FT_DERIV;
  END_VAR

  (* run differentiator *)
  diff(IN := IN, K := TV);

  (* combine both values *)
  Y := KP * (diff.out + IN);

  (* From OSCAT Library, www.oscat.de *)
  (* FT_DERIV required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK FT_PDT1
  VAR_INPUT
    IN : REAL;
    KP : REAL := 1.0;
    TV : REAL := 1.0;
    T1 : REAL := 1.0;
  END_VAR
  VAR_OUTPUT
    Y : REAL;
  END_VAR
  VAR
    diff : FT_DERIV;
    _TP : FT_PT1;
  END_VAR

  (* run differentiator *)
  diff(IN := IN, K := TV);

  (* Run PT1 filter *)
  _tp(in := diff.out, _T := REAL_TO_TIME(T1));

  (* combine both values *)
  Y := KP * (_tp.out + IN);

  (* From OSCAT Library, www.oscat.de *)
  (* T_PLC_US required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK FT_PI
  VAR_INPUT
    IN : REAL;
    KP : REAL := 1.0;
    KI : REAL := 1.0;
    ILIM_L : REAL := -1.0e38;
    ILIM_H : REAL := 1.0e38;
    IEN : BOOL := TRUE;
    RST : BOOL := FALSE;
  END_VAR
  VAR_OUTPUT
    Y : REAL;
    LIM : BOOL;
  END_VAR
  VAR
    integ : FT_INT;
  END_VAR

  (* run integrator *)
  integ(IN := IN, K := KI, RUN := IEN, RST := RST, OUT_MIN := ILIM_L, OUT_MAX := ILIM_H);
  (* check if integrator has reached its limits and set overflow *)
  LIM := integ.LIM;
  Y := KP * IN + integ.Out;

  (* From OSCAT Library, www.oscat.de *)
  (* FT_INT required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK FT_PT2
  VAR_INPUT
    IN : REAL;
    _T : TIME;
    _D : REAL;
    K : REAL := 1.0;
  END_VAR
  VAR_OUTPUT
    OUT : REAL;
  END_VAR
  VAR
    init : BOOL;
    int1 : INTEGRATE;
    int2 : INTEGRATE;
    tn : REAL;
    I1 : REAL;
    I2 : REAL;
    tn2 : REAL;
  END_VAR

  (* startup initialisation *)
  IF NOT init OR _T = T#0s THEN
  	init := TRUE;
  	out := K * in;
  	I2 := out;
  ELSE
  	TN := TIME_TO_REAL(_T) / 1000.0;
  	tn2 := TN * TN;
  	int1(X := in * K / tn2 - I1 * 0.5 * _D / TN - I2 / TN2, Y := I1);
  	I1 := int1.Y;
  	int2(X := I1,Y := I2);
  	I2 := int2.Y;
  	out := I2;
  END_IF;

  (* From OSCAT Library, www.oscat.de *)
  (* INTEGRATE required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK FT_TN16
  VAR_INPUT
    IN : REAL;
    _T : TIME;
  END_VAR
  VAR_OUTPUT
    OUT : REAL;
    TRIG : BOOL;
  END_VAR
  VAR
    length : INT := 16;
    X : ARRAY [0..15] OF REAL;
    cnt : INT;
    last : TIME;
    tx : TIME;
    init : BOOL;
  END_VAR

  tx:= UDINT_TO_TIME(T_PLC_MS(en:=true));

  trig := FALSE;
  IF NOT init THEN
  	x[cnt] := in;
  	init := TRUE;
  	last := tx;
  ELSIF tx - last >= _T / length THEN
  	IF cnt = length - 1 THEN cnt := 0; ELSE cnt := cnt + 1; END_IF;
  	Out := X[cnt];
  	x[cnt] := in;
  	last := tx;
  	Trig := TRUE;
  END_IF;

  (* From OSCAT Library, www.oscat.de *)
  (* T_PLC_MS required *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK HYST
  VAR_INPUT
    IN : REAL;
    _ON : REAL;
    OFF : REAL;
  END_VAR
  VAR_OUTPUT
    Q : BOOL;
    WIN : BOOL;
  END_VAR

  IF _ON >= OFF THEN

  	IF IN < OFF THEN

  		Q := FALSE;

  		WIN := FALSE;

  	ELSIF  IN > _ON THEN

  		Q := TRUE;

  		WIN := FALSE;

  	ELSE

  		WIN := TRUE;

  	END_IF;

  ELSE

  	IF IN > OFF THEN

  		Q := FALSE;

  		WIN := FALSE;

  	ELSIF  IN < _ON THEN

  		Q := TRUE;

  		WIN := FALSE;

  	ELSE

  		WIN := TRUE;

  	END_IF;

  END_IF;


  (* From OSCAT Library, www.oscat.de *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK HYST_1
  VAR_INPUT
    IN : REAL;
    HIGH : REAL;
    LOW : REAL;
  END_VAR
  VAR_OUTPUT
    Q : BOOL;
    WIN : BOOL;
  END_VAR

  IF in < low THEN

  	Q := FALSE;

  	win := FALSE;

  ELSIF in > high THEN

  	Q := TRUE;

  	win := FALSE;

  ELSE

  	win := TRUE;

  END_IF;


  (* From OSCAT Library, www.oscat.de *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK HYST_2
  VAR_INPUT
    IN : REAL;
    VAL : REAL;
    HYS : REAL;
  END_VAR
  VAR_OUTPUT
    Q : BOOL;
    WIN : BOOL;
  END_VAR
  VAR
    tmp : REAL;
  END_VAR

  tmp := val - hys * 0.5;

  IF in < tmp THEN

  	Q := FALSE;

  	win := FALSE;

  ELSIF in > tmp + hys THEN

  	Q := TRUE;

  	win := FALSE;

  ELSE

  	win := TRUE;

  END_IF;


  (* From OSCAT Library, www.oscat.de *)
END_FUNCTION_BLOCK

FUNCTION_BLOCK HYST_3
  VAR_INPUT
    IN : REAL;
    _HYST : REAL;
    VAL1 : REAL;
    VAL2 : REAL;
  END_VAR
  VAR_OUTPUT
    Q1 : BOOL;
    Q2 : BOOL;
  END_VAR
  VAR
    X : REAL;
  END_VAR

  X := _hyst * 0.5;

  IF in < val1 - X THEN

  	q1 := TRUE;

  ELSIF in > val1 + X THEN

  	q1 := FALSE;

  END_IF;

  IF in < val2 - X THEN

  	q2 := FALSE;

  ELSIF in > val2 + X THEN

  	q2 := TRUE;

  END_IF;


  (* From OSCAT Library, www.oscat.de *)
END_FUNCTION_BLOCK


CONFIGURATION Config0

  RESOURCE Res0 ON PLC
    TASK task0(INTERVAL := T#20ms,PRIORITY := 0);
    PROGRAM instance0 WITH task0 : program0;
  END_RESOURCE
END_CONFIGURATION

(*DBG:char md5[] = "c1e729d9e5586f943f3211db7ba92edb";*)
(*DBG:/**)
(*DBG: * This file is part of OpenPLC Runtime*)
(*DBG: **)
(*DBG: * Copyright (C) 2023 Autonomy, GP Orcullo*)
(*DBG: * Based on the work by GP Orcullo on Beremiz for uC*)
(*DBG: **)
(*DBG: * This program is free software; you can redistribute it and/or*)
(*DBG: * modify it under the terms of the GNU General Public License*)
(*DBG: * as published by the Free Software Foundation; either version 2*)
(*DBG: * of the License, or (at your option) any later version.*)
(*DBG: **)
(*DBG: * This program is distributed in the hope that it will be useful,*)
(*DBG: * but WITHOUT ANY WARRANTY; without even the implied warranty of*)
(*DBG: * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the*)
(*DBG: * GNU General Public License for more details.*)
(*DBG: **)
(*DBG: * You should have received a copy of the GNU General Public License*)
(*DBG: * along with this program; If not, see <http://www.gnu.org/licenses/>.*)
(*DBG: **)
(*DBG: */*)
(*DBG:*)
(*DBG:#include <stdbool.h>*)
(*DBG:*)
(*DBG:#include "iec_types_all.h"*)
(*DBG:#include "POUS.h"*)
(*DBG:*)
(*DBG:#define SAME_ENDIANNESS      0*)
(*DBG:#define REVERSE_ENDIANNESS   1*)
(*DBG:*)
(*DBG:uint8_t endianness;*)
(*DBG:*)
(*DBG:*)
(*DBG:extern PROGRAM0 RES0__INSTANCE0;*)
(*DBG:*)
(*DBG:static const struct {*)
(*DBG:    void *ptr;*)
(*DBG:    __IEC_types_enum type;*)
(*DBG:} debug_vars[] = {*)
(*DBG:    {&(RES0__INSTANCE0.ASKED_TEMPERATURE), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.FINAL_TEMPERATURE), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.FINAL_TEMPERATURE_1), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.POWER_FAN), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.PRESSURE_DIFFERENCE), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.POWER_FAN_SENT), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.VOLTAGE_SENT), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.VOLTAGE_12V), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.VOLTAGE_LIMIT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._100), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.ONOFF), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.ONOFF_FAN), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.P), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.I), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._D), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._0), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TEST), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.TEST0), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.ACT), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.SET), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.SUP), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.OFS), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.M_I), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.MAN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.RST), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.KP), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.TN), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.TV), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.LL), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.LH), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.Y), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.DIFF), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.LIM), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.IN), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.KP), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.TN), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.TV), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.LIM_L), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.LIM_H), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.RST), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.Y), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.LIM), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.PIWL.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.PIWL.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.PIWL.IN), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.PIWL.KP), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.PIWL.KI), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.PIWL.LIM_L), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.PIWL.LIM_H), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.PIWL.RST), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.PIWL.Y), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.PIWL.LIM), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.PIWL.INIT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.PIWL.TX), UDINT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.PIWL.TC), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.PIWL.T_LAST), UDINT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.PIWL.IN_LAST), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.PIWL.I), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.PIWL.P), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.DIFF.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.DIFF.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.DIFF.IN), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.DIFF.K), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.DIFF.RUN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.DIFF.OUT), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.DIFF.OLD), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.DIFF.TX), UDINT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.DIFF.LAST), UDINT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.DIFF.INIT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0._PID.DIFF.TC), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.CO.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.CO.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.CO.CI), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.CO.OFFSET), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.CO.MAN_IN), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.CO.LIM_L), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.CO.LIM_H), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.CO.MANUAL), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.CO.Y), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.CTRL_PID0.CO.LIM), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.VARIABLE), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_INT_TO_REAL46_OUT), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_DIV14_OUT), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_INT_TO_REAL1_OUT), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_INT_TO_BOOL3_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_NOT84_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_INT_TO_BOOL76_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_AND83_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_NOT81_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_MUL4_OUT), REAL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_REAL_TO_INT59_OUT), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_REAL_TO_INT6_OUT), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_GE91_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_BOOL_TO_INT92_OUT), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_MUL93_OUT), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_REAL_TO_INT88_OUT), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_LE86_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_BOOL_TO_INT9_OUT), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_MUL8_OUT), INT_ENUM},*)
(*DBG:};*)
(*DBG:*)
(*DBG:#define VAR_COUNT               103*)
(*DBG:*)
(*DBG:uint16_t get_var_count(void)*)
(*DBG:{*)
(*DBG:    return VAR_COUNT;*)
(*DBG:}*)
(*DBG:*)
(*DBG:size_t get_var_size(size_t idx)*)
(*DBG:{*)
(*DBG:    if (idx >= VAR_COUNT)*)
(*DBG:    {*)
(*DBG:        return 0;*)
(*DBG:    }*)
(*DBG:    switch (debug_vars[idx].type) {*)
(*DBG:    case REAL_ENUM:*)
(*DBG:        return sizeof(REAL);*)
(*DBG:    case INT_ENUM:*)
(*DBG:        return sizeof(INT);*)
(*DBG:    case BOOL_ENUM:*)
(*DBG:        return sizeof(BOOL);*)
(*DBG:    case UDINT_ENUM:*)
(*DBG:        return sizeof(UDINT);*)
(*DBG:    default:*)
(*DBG:        return 0;*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void *get_var_addr(size_t idx)*)
(*DBG:{*)
(*DBG:    void *ptr = debug_vars[idx].ptr;*)
(*DBG:*)
(*DBG:    switch (debug_vars[idx].type) {*)
(*DBG:    case REAL_ENUM:*)
(*DBG:        return (void *)&((__IEC_REAL_t *) ptr)->value;*)
(*DBG:    case INT_ENUM:*)
(*DBG:        return (void *)&((__IEC_INT_t *) ptr)->value;*)
(*DBG:    case BOOL_ENUM:*)
(*DBG:        return (void *)&((__IEC_BOOL_t *) ptr)->value;*)
(*DBG:    case UDINT_ENUM:*)
(*DBG:        return (void *)&((__IEC_UDINT_t *) ptr)->value;*)
(*DBG:    default:*)
(*DBG:        return 0;*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void force_var(size_t idx, bool forced, void *val)*)
(*DBG:{*)
(*DBG:    void *ptr = debug_vars[idx].ptr;*)
(*DBG:*)
(*DBG:    if (forced) {*)
(*DBG:        size_t var_size = get_var_size(idx);*)
(*DBG:        switch (debug_vars[idx].type) {*)
(*DBG:        case REAL_ENUM: {*)
(*DBG:            memcpy(&((__IEC_REAL_t *) ptr)->value, val, var_size);*)
(*DBG:            //((__IEC_REAL_t *) ptr)->value = *((REAL *) val);*)
(*DBG:            ((__IEC_REAL_t *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        case INT_ENUM: {*)
(*DBG:            memcpy(&((__IEC_INT_t *) ptr)->value, val, var_size);*)
(*DBG:            //((__IEC_INT_t *) ptr)->value = *((INT *) val);*)
(*DBG:            ((__IEC_INT_t *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        case BOOL_ENUM: {*)
(*DBG:            memcpy(&((__IEC_BOOL_t *) ptr)->value, val, var_size);*)
(*DBG:            //((__IEC_BOOL_t *) ptr)->value = *((BOOL *) val);*)
(*DBG:            ((__IEC_BOOL_t *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        case UDINT_ENUM: {*)
(*DBG:            memcpy(&((__IEC_UDINT_t *) ptr)->value, val, var_size);*)
(*DBG:            //((__IEC_UDINT_t *) ptr)->value = *((UDINT *) val);*)
(*DBG:            ((__IEC_UDINT_t *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        default:*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:    } else {*)
(*DBG:        switch (debug_vars[idx].type) {*)
(*DBG:        case REAL_ENUM:*)
(*DBG:            ((__IEC_REAL_t *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        case INT_ENUM:*)
(*DBG:            ((__IEC_INT_t *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        case BOOL_ENUM:*)
(*DBG:            ((__IEC_BOOL_t *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        case UDINT_ENUM:*)
(*DBG:            ((__IEC_UDINT_t *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        default:*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void swap_bytes(void *ptr, size_t size) *)
(*DBG:{*)
(*DBG:    uint8_t *bytePtr = (uint8_t *)ptr;*)
(*DBG:    size_t i;*)
(*DBG:    for (i = 0; i < size / 2; ++i) *)
(*DBG:    {*)
(*DBG:        uint8_t temp = bytePtr[i];*)
(*DBG:        bytePtr[i] = bytePtr[size - 1 - i];*)
(*DBG:        bytePtr[size - 1 - i] = temp;*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void trace_reset(void)*)
(*DBG:{*)
(*DBG:    for (size_t i=0; i < VAR_COUNT; i++) *)
(*DBG:    {*)
(*DBG:        force_var(i, false, 0);*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void set_trace(size_t idx, bool forced, void *val)*)
(*DBG:{*)
(*DBG:    if (idx >= 0 && idx < VAR_COUNT) *)
(*DBG:    {*)
(*DBG:        if (endianness == REVERSE_ENDIANNESS)*)
(*DBG:        {*)
(*DBG:            // Aaaaarghhhh... Stupid AVR is Big Endian.*)
(*DBG:            swap_bytes(val, get_var_size(idx));*)
(*DBG:        }*)
(*DBG:*)
(*DBG:        force_var(idx, forced, val);*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void set_endianness(uint8_t value)*)
(*DBG:{*)
(*DBG:    if (value == SAME_ENDIANNESS || value == REVERSE_ENDIANNESS)*)
(*DBG:    {*)
(*DBG:        endianness = value;*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
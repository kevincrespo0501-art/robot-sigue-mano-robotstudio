MODULE SeguirMano
    !=====================================================================
    ! SeguirMano.mod  -  El robot sigue la mano detectada por la webcam.
    !
    ! Servidor TCP en el controlador (virtual) de RobotStudio.
    ! Python (seguir_mano_robot.py) envia el OBJETIVO mas reciente:
    !     "M;dx;dy;dz;"  -> objetivo = HOME + (dx,dy,dz) en mm (wobj0)
    !     "H;"           -> objetivo = HOME
    ! y el robot contesta "OK" en cuanto lo recibe (sin esperar a moverse).
    !
    ! Bucle de seguimiento (servo):
    !   1. mira si ha llegado un objetivo nuevo (espera como maximo 20 ms)
    !   2. avanza un PASO corto (max. PASO mm) hacia el ultimo objetivo
    !   3. repite
    ! Asi el robot siempre va hacia donde esta la mano AHORA y nunca
    ! ejecuta posiciones viejas: el retardo no se acumula.
    !
    ! HOME = postura jHome (o la posicion actual si USAR_JHOME = FALSE).
    !=====================================================================

    ! --- Comunicacion ---------------------------------------------------
    CONST string IP := "127.0.0.1";   ! controlador virtual en el mismo PC
    CONST num PUERTO := 1025;

    ! --- Limites de seguridad (mm respecto a HOME) ------------------------
    CONST num LIM_X := 150;
    CONST num LIM_Y := 300;
    CONST num LIM_Z := 200;

    ! --- Postura HOME (ejes, grados). Pensada para el GoFa CRB 15000:
    !     brazo adelantado, muneca doblada y lejos de singularidades.
    !     Pon USAR_JHOME := FALSE para usar la posicion actual del robot.
    CONST bool USAR_JHOME := TRUE;
    CONST jointtarget jHome := [[0, 15, 25, 0, 50, 0], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];

    ! --- Movimiento -------------------------------------------------------
    CONST num PASO := 40;              ! mm maximos por movimiento
    CONST num TOL := 2;                ! mm: "ya he llegado"
    VAR speeddata vel := [1000, 500, 5000, 1000];
    VAR zonedata zona := z10;          ! menor que PASO/2: enlaza los pasos

    VAR socketdev servidor;
    VAR socketdev cliente;
    VAR string msg;
    VAR bool hayMsg;
    VAR robtarget pHome;
    VAR robtarget pObj;                ! objetivo mas reciente (la mano)
    VAR robtarget pCmd;                ! ultimo punto mandado al robot
    VAR num dx;
    VAR num dy;
    VAR num dz;

    PROC main()
        ConfL \Off;
        SingArea \Wrist;
        IF USAR_JHOME MoveAbsJ jHome, v300, fine, tool0;
        pHome := CRobT(\Tool:=tool0 \WObj:=wobj0);
        pObj := pHome;
        pCmd := pHome;
        TPErase;
        TPWrite "SeguirMano: HOME guardado";
        AbrirServidor;
        WHILE TRUE DO
            TPWrite "Esperando a Python en el puerto " \Num:=PUERTO;
            SocketAccept servidor, cliente \Time:=WAIT_MAX;
            TPWrite "Python conectado";
            Atender;
            SocketClose cliente;
        ENDWHILE
    ENDPROC

    PROC AbrirServidor()
        ! Por si quedo abierto de una ejecucion anterior
        SocketClose servidor;
        SocketCreate servidor;
        SocketBind servidor, IP, PUERTO;
        SocketListen servidor;
    ENDPROC

    PROC Atender()
        WHILE TRUE DO
            ! 1. Leer un objetivo nuevo, si lo hay (espera max. 20 ms)
            hayMsg := TRUE;
            SocketReceive cliente \Str:=msg \Time:=0.02;
            IF hayMsg THEN
                Procesar;
                SocketSend cliente \Str:="OK";
            ENDIF
            ! 2. Dar un paso corto hacia el objetivo mas reciente
            Avanzar;
        ENDWHILE
    ERROR
        IF ERRNO = ERR_SOCK_TIMEOUT THEN
            hayMsg := FALSE;           ! no ha llegado nada: seguimos
            TRYNEXT;
        ELSEIF ERRNO = ERR_SOCK_CLOSED THEN
            TPWrite "Python se ha desconectado";
            RETURN;
        ENDIF
    ENDPROC

    PROC Procesar()
        VAR num p1;
        VAR num p2;
        VAR num p3;
        VAR num p4;
        VAR bool ok;

        IF StrPart(msg, 1, 1) = "H" THEN
            pObj := pHome;
            RETURN;
        ENDIF
        IF StrPart(msg, 1, 1) <> "M" RETURN;

        ! Formato  M;dx;dy;dz;
        p1 := StrFind(msg, 1, ";");
        p2 := StrFind(msg, p1 + 1, ";");
        p3 := StrFind(msg, p2 + 1, ";");
        p4 := StrFind(msg, p3 + 1, ";");
        IF p4 > StrLen(msg) RETURN;
        ok := StrToVal(StrPart(msg, p1 + 1, p2 - p1 - 1), dx);
        ok := ok AND StrToVal(StrPart(msg, p2 + 1, p3 - p2 - 1), dy);
        ok := ok AND StrToVal(StrPart(msg, p3 + 1, p4 - p3 - 1), dz);
        IF NOT ok RETURN;

        dx := Limitar(dx, LIM_X);
        dy := Limitar(dy, LIM_Y);
        dz := Limitar(dz, LIM_Z);
        pObj := Offs(pHome, dx, dy, dz);
    ENDPROC

    PROC Avanzar()
        VAR num d;
        d := Distance(pCmd.trans, pObj.trans);
        IF d < TOL RETURN;             ! ya estamos en la mano
        IF d > PASO THEN
            pCmd.trans := pCmd.trans + (pObj.trans - pCmd.trans) * (PASO / d);
        ELSE
            pCmd.trans := pObj.trans;
        ENDIF
        MoveL pCmd, vel, zona, tool0 \WObj:=wobj0;
    ENDPROC

    FUNC num Limitar(num v, num lim)
        IF v > lim RETURN lim;
        IF v < -lim RETURN -lim;
        RETURN v;
    ENDFUNC
ENDMODULE

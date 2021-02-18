package PowerSysPro
  //Copyright 2020 EDF
  extends Icons.myPackage;
  import CM = PowerSysPro.Functions;

  package Components "Components for load flow calculations"
    model mySource "Perfect voltage source"
      extends Icons.mySource;
      Interfaces.myAcausalTerminal terminal(v(re(start = UNom / sqrt(3), nominal = UNom / sqrt(3)), im(start = 0))) "Terminal of the node" annotation (
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    public
      parameter Types.myVoltage UNom "Reference voltage of the source";
      parameter Types.myAngle theta = 0 "Phase shift of voltage phasor";
      Types.myCurrent I = CM.abs(terminal.i) "Current flowing the source";
      Types.myVoltage U = sqrt(3) * CM.abs(terminal.v) "Voltage at the source";
      Types.myApparentPower S = sqrt(3) * I * U "Apparent power flowing the source";
    equation
      terminal.v = CM.fromPolar(UNom / sqrt(3), theta);
      annotation (
        Icon(coordinateSystem(grid = {0.1, 0.1}, initialScale = 0.1)),
        Documentation(info = "<html>
    <p>This node prescribes the voltage magnitude and phase (default zero) for the source.</p>
</html>"));
    end mySource;

    model myPVNode "PV node for islanding"
      extends Icons.myPV;
      Interfaces.myAcausalTerminal terminal(v(re(start = UNom / sqrt(3), nominal = UNom / sqrt(3)), im(start = 0))) "Terminal of the node" annotation (
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    public
      parameter Types.myVoltage UNom "Reference voltage of the PV node";
      //parameter Types.myActivePower Pstart "Starting active power to provide (<0)";
      parameter Types.myActivePower Pmax "Maximum active power to provide (<0)";
      Types.myActivePower P "Active power flowing the node";
      Types.myReactivePower Q "Reactive power flowing the node";
      Types.myCurrent I = CM.abs(terminal.i) "Current flowing the PV node";
      Types.myVoltage U = sqrt(3) * CM.abs(terminal.v) "Voltage at the PV node";
      Types.myApparentPower S = sqrt(3) * I * U "Apparent power flowing the PV node";
    equation
      assert(not (-P) < (-Pmax), ">>> Active power is below the maximum acceptable value for " + getInstanceName(), AssertionLevel.warning);
      assert(not (-P) > (-Pmax), ">>> Active power is above the maximum acceptable value for " + getInstanceName(), AssertionLevel.warning);
      //3 * terminal.v * CM.conj(terminal.i) = Complex(P, Q);  //do not operate well with Dymola!
      terminal.i = CM.conj(Complex(P, Q) / (3 * terminal.v));
      terminal.v = CM.fromPolar(UNom / sqrt(3), atan(Q / P));
      annotation (
        Icon(coordinateSystem(grid = {0.1, 0.1}, initialScale = 0.1, preserveAspectRatio = false), graphics={  Text(origin = {60.1, -63.1}, lineColor = {0, 0, 255}, extent = {{-124, 11}, {124, -11}}, textString = if OnePhaseLoad then "1" else "3", textStyle = {TextStyle.Bold}), Text(extent = {{-40.5, -6}, {41.5, -46}}, lineColor = {0, 0, 0}, textStyle = {TextStyle.Bold}, textString = "~")}),
        Documentation(info = "<html>
    <p>This node prescribes the constant active power <code>P</code> entering the PV bus.</p>
    </html>"),
        Diagram(coordinateSystem(preserveAspectRatio = false)));
    end myPVNode;

    model myLoad "Load with P/Q as fixed values"
      extends PartialModels.myPartialLoad;
    public
      parameter Types.myActivePower P "Active power consumed (>0) or provided (<0)";
      parameter Types.myReactivePower Q = 0 "Reactive power inductive (>0) or capacitive (<0)";
    equation
      3 * terminal.v * CM.conj(terminal.i) = Complex(P, Q);
      annotation (
        Icon(coordinateSystem(grid = {0.1, 0.1}, initialScale = 0.1), graphics={  Text(origin = {60.1, -63.1}, lineColor = {0, 0, 255}, extent = {{-124, 11}, {124, -11}}, textString = if OnePhaseLoad then "1" else "3", textStyle = {TextStyle.Bold})}),
        Documentation(info = "<html>
    <p>This node prescribes a constant active power <code>P</code> and reactive power <code>Q</code> entering the PQ bus.</p>
    </html>"));
    end myLoad;

    model myVariableLoad "Load with P/Q as input variables"
      extends PartialModels.myPartialLoad;
      Modelica.Blocks.Interfaces.RealInput PInput(final unit = "W", displayUnit = "W") "Active power consumed (>0) or provided (<0)" annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 0, origin = {-100, 40}), iconTransformation(extent = {{-15.3, -15.3}, {15.3, 15.3}}, rotation = 90, origin = {-25.4, -84.5})));
      Modelica.Blocks.Interfaces.RealInput QInput(final unit = "var", displayUnit = "var") "Reactive power inductive (>0) or capacitive (<0)" annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 0, origin = {-100, -40}), iconTransformation(extent = {{-15.85, -15.85}, {15.85, 15.85}}, rotation = 90, origin = {25.65, -84.35})));
      parameter Boolean switchToImpedanceMode = true "Possibility to switch to impedance mode when voltage is too low";
    protected
      Boolean degradedMode(start = false, fixed = true) "Mode for load: normal / degraded";
      Types.myComplexAdmittance Y = Complex(PInput, QInput) / minU ^ 2 "Admittance of the load";
    equation
      degradedMode = if U < minU and switchToImpedanceMode then true else false;
      when degradedMode <> pre(degradedMode) then
        assert(degradedMode == false, ">>> Switching to the impedance mode for " + getInstanceName(), AssertionLevel.warning);
        assert(degradedMode, ">>> Normal mode assumed for " + getInstanceName(), AssertionLevel.warning);
      end when;
      if degradedMode then
        terminal.i = Y * terminal.v;
      else
        3 * terminal.v * CM.conj(terminal.i) = Complex(PInput, QInput);
      end if;
      annotation (
        Icon(coordinateSystem(grid = {0.1, 0.1}, initialScale = 0.1), graphics={  Text(origin = {60.1, -63.1}, lineColor = {0, 0, 255}, extent = {{-124, 11}, {124, -11}}, textString = if OnePhaseLoad then "1" else "3", textStyle = {TextStyle.Bold})}),
        Documentation(info = "<html>
    <p>This node prescribes a variable active power <code>P</code> and reactive power <code>Q</code> entering the PQ bus.</p>
    </html>"));
    end myVariableLoad;

    model myCapacitorBank "Capacitor bank with fixed capacitance"
      extends Icons.myCapacitorBank;
      extends Icons.myOnePortAC;
      Interfaces.myAcausalTerminal terminal(v(re(start = UNom / sqrt(3), nominal = UNom / sqrt(3)), im(start = 0))) "Terminal of the node" annotation (
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    public
      parameter Types.myVoltage UNom "Reference voltage of the capacitor bank";
      parameter Types.myConductance G = 0 "Capacitor bank conductance";
      parameter Types.mySusceptance B "Capacitor bank susceptance";
      Types.myCurrent I = CM.abs(terminal.i) "Current flowing the capacitor bank";
      Types.myVoltage U = sqrt(3) * CM.abs(terminal.v) "Voltage at the capacitor bank";
      Types.myApparentPower S = sqrt(3) * I * U "Apparent power flowing the capacitor bank";
    protected
      parameter Types.myComplexAdmittance Y = Complex(G, B) "Capacitor bank admittance";
    equation
      terminal.i = terminal.v * Y;
      annotation (
        Icon(coordinateSystem(grid = {0.1, 0.1}, initialScale = 0.1), graphics={  Text(origin = {60.1, -63.1}, lineColor = {0, 0, 255}, extent = {{-124, 11}, {124, -11}}, textString = if OnePhaseLoad then "1" else "3", textStyle = {TextStyle.Bold})}),
        Documentation(info = "<html>
    <p>This node prescribes a capacitor bank with fixed conductance and/or susceptance.</p>
    </html>"));
    end myCapacitorBank;

    model myLine "MV or LV line with constant impedance"
      extends Icons.myLine;
      extends Icons.myTwoPortsAC;
      Interfaces.myAcausalTerminal terminalA(i(re(start = 0), im(start = 0))) "Terminal A of the node" annotation (
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Interfaces.myAcausalTerminal terminalB(v(re(start = UNom / sqrt(3), nominal = UNom / sqrt(3)), im(start = 0))) "Terminal B of the node" annotation (
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    public
      parameter Types.myVoltage UNom "Reference voltage of the line";
      parameter Types.myResistance R "Series resistance of phase conductor";
      parameter Types.myReactance X = 0 "Series reactance of phase conductor";
      parameter Types.myConductance G = 0 "Shunt conductance of phase conductor coupling";
      parameter Types.mySusceptance B = 0 "Shunt susceptance of phase conductor coupling";
      parameter Types.myPerUnit l = 1 "lineic coefficient";
      parameter Types.myCurrent Imax = 0 "Maximum admissible current. 0 means no control";
      Types.myCurrent IA = CM.abs(terminalA.i) "Current flowing port A";
      Types.myVoltage UA = sqrt(3) * CM.abs(terminalA.v) "Voltage at port A";
      Types.myCurrent IB = CM.abs(terminalB.i) "Current flowing port B";
      Types.myVoltage UB = sqrt(3) * CM.abs(terminalB.v) "Voltage at port B";
      Types.myApparentPower S = sqrt(3) * IA * UA "Apparent power flowing port A";
    protected
      parameter Types.myComplexImpedance Z = Complex(l * R, l * X) "Series impedance of phase conductor";
      parameter Types.myComplexAdmittance Y = Complex(l * G / 2, l * B / 2) "Shunt admittance at port A and port B";
    equation
      if Imax > 0 then
        assert(IA <= Imax, ">>> Current flowing the line is exceeding the maximum admissible current for " + getInstanceName(), AssertionLevel.warning);
      end if;
      terminalA.i + terminalB.i = Y * (terminalB.v + terminalA.v);
      terminalA.v - terminalB.v = Z * (terminalA.i - Y * terminalA.v);
      annotation (
        Documentation(info = "<html>
    <p>A line can be modeled as a dedicated Pi-network. A diagram of the model is shown in the next figure:</p>
    <p>  </p>
    <figure> <img src=\"modelica://PowerSysPro/Resources/Images/PiNetworkLine.png\"></figure>
</html>"));
    end myLine;

    model myLightLine "MV or LV line without possible check of the maximum admissible current"
      extends Icons.myLine;
      extends Icons.myTwoPortsAC;
      Interfaces.myAcausalTerminal terminalA(i(re(start = 0), im(start = 0))) "Terminal A of the node" annotation (
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Interfaces.myAcausalTerminal terminalB(v(re(start = UNom / sqrt(3), nominal = UNom / sqrt(3)), im(start = 0))) "Terminal B of the node" annotation (
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    public
      parameter Types.myVoltage UNom "Reference voltage of the line";
      parameter Types.myResistance R "Series resistance of phase conductor";
      parameter Types.myReactance X = 0 "Series reactance of phase conductor";
      parameter Types.myConductance G = 0 "Shunt conductance of phase conductor coupling";
      parameter Types.mySusceptance B = 0 "Shunt susceptance of phase conductor coupling";
      parameter Types.myPerUnit l = 1 "lineic coefficient";
      Types.myCurrent IA = CM.abs(terminalA.i) "Current flowing port A";
      Types.myVoltage UA = sqrt(3) * CM.abs(terminalA.v) "Voltage at port A";
      Types.myCurrent IB = CM.abs(terminalB.i) "Current flowing port B";
      Types.myVoltage UB = sqrt(3) * CM.abs(terminalB.v) "Voltage at port B";
      Types.myApparentPower S = sqrt(3) * IA * UA "Apparent power flowing port A";
    protected
      parameter Types.myComplexImpedance Z = Complex(l * R, l * X) "Series impedance of phase conductor";
      parameter Types.myComplexAdmittance Y = Complex(l * G / 2, l * B / 2) "Shunt admittance at port A and port B";
    equation
      terminalA.i + terminalB.i = Y * (terminalB.v + terminalA.v);
      terminalA.v - terminalB.v = Z * (terminalA.i - Y * terminalA.v);
      annotation (
        Documentation(info = "<html>
    <p>A line can be modeled as a dedicated Pi-network. A diagram of the model is shown in the next figure:</p>
    <p>  </p>
    <figure> <img src=\"modelica://PowerSysPro/Resources/Images/PiNetworkLine.png\"></figure>
</html>"));
    end myLightLine;

    model myLineWithFault "MV or LV line with constant impedance and fault at intermediate position"
      extends Icons.myLine;
      extends Icons.myTwoPortsAC;
      extends Icons.myFault;
      Interfaces.myAcausalTerminal terminalA(i(re(start = 0), im(start = 0))) "Terminal A of the node" annotation (
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Interfaces.myAcausalTerminal terminalB(v(re(start = UNom / sqrt(3), nominal = UNom / sqrt(3)), im(start = 0))) "Terminal B of the node" annotation (
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      myLightLine lineA(UNom = UNom, R = faultLocationPu * l * R, X = faultLocationPu * l * X, B = faultLocationPu * l * B, G = faultLocationPu * l * G) annotation (
        Placement(visible = true, transformation(origin = {-40, 2.44249e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      myLightLine lineB(UNom = UNom, R = (1 - faultLocationPu) * l * R, X = (1 - faultLocationPu) * l * X, B = (1 - faultLocationPu) * l * B, G = (1 - faultLocationPu) * l * G) annotation (
        Placement(visible = true, transformation(origin = {40, 1.77636e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      PartialModels.myFault myFault(R = RFault, X = XFault, startTime = startTime, stopTime = stopTime) annotation (
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-22, -22}, {22, 22}}, rotation = 0)));
    public
      parameter Types.myVoltage UNom "Reference voltage of the line";
      parameter Types.myResistance R "Series resistance of phase conductor";
      parameter Types.myReactance X = 0 "Series reactance of phase conductor";
      parameter Types.myConductance G = 0 "Shunt conductance of phase conductor coupling";
      parameter Types.mySusceptance B = 0 "Shunt susceptance of phase conductor coupling";
      parameter Types.myPerUnit l = 1 "lineic coefficient";
      parameter Types.myCurrent Imax = 0 "Maximum admissible current. 0 means no control";
      parameter Types.myPerUnit faultLocationPu "Location of the fault (0: port A, 1: port B)" annotation (
        Dialog(group = "Fault data"));
      parameter Types.myResistance RFault "Fault resistance" annotation (
        Dialog(group = "Fault data"));
      parameter Types.myReactance XFault = 0 "Fault reactance" annotation (
        Dialog(group = "Fault data"));
      parameter Types.myTime startTime "Start time of the fault" annotation (
        Dialog(group = "Fault data"));
      parameter Types.myTime stopTime "End time of the fault" annotation (
        Dialog(group = "Fault data"));
      Types.myCurrent IA = CM.abs(terminalA.i) "Current flowing port A";
      Types.myVoltage UA = sqrt(3) * CM.abs(terminalA.v) "Voltage at port A";
      Types.myCurrent IB = CM.abs(terminalB.i) "Current flowing port B";
      Types.myVoltage UB = sqrt(3) * CM.abs(terminalB.v) "Voltage at port B";
      Types.myApparentPower S = sqrt(3) * IA * UA "Apparent power flowing port A";
    protected
      parameter Types.myComplexImpedance Z = Complex(l * R, l * X) "Series impedance of phase conductor";
      parameter Types.myComplexAdmittance Y = Complex(l * G / 2, l * B / 2) "Shunt admittance at port A and port B";
    equation
      if Imax > 0 then
        assert(IA <= Imax, ">>> Current flowing the line is exceeding the maximum admissible current for " + getInstanceName(), AssertionLevel.warning);
      end if;
      connect(lineB.terminalB, terminalB) annotation (
        Line(points = {{60, 0}, {96, 0}, {96, 0}, {100, 0}}));
      connect(lineA.terminalA, terminalA) annotation (
        Line(points = {{-60, 3.55271e-15}, {-100, 3.55271e-15}, {-100, 0}}));
      connect(lineA.terminalB, myFault.terminal) annotation (
        Line(points = {{-20, 0}, {-3.10862e-15, 0}}, color = {0, 0, 0}));
      connect(myFault.terminal, lineB.terminalA) annotation (
        Line(points = {{-3.10862e-15, 0}, {20, 0}}, color = {0, 0, 0}));
      connect(terminalA, terminalA) annotation (
        Line(points = {{-100, 0}, {-100, 0}}, color = {0, 0, 0}));
      annotation (
        Documentation(info = "<html>
    <p>A line can be modeled as a dedicated Pi-network. A diagram of the model is shown in the next figure:</p>
    <p>  </p>
    <figure> <img src=\"modelica://PowerSysPro/Resources/Images/PiNetworkLine.png\"></figure>
</html>"));
    end myLineWithFault;

    model myTransformer "Transformer with fixed voltage ratio"
      extends PartialModels.myPartialTransformer;
    protected
      parameter Complex k = CM.fromPolar(UNomB / UNomA, theta) "Complex ratio of ideal transformer";
      parameter Types.myComplexImpedance Z = Complex(R, X) "Series impedance of phase conductor";
      parameter Types.myComplexAdmittance Y = Complex(G, B) "Shunt admittance at primary side";
    equation
      assert(UNomB / UNomA < 1, "Voltage at port A should be higher then voltage at port B for " + getInstanceName());
      terminalA.i + terminalB.i * CM.conj(k) = Y * terminalA.v;
      terminalA.v - terminalB.v / k = -Z * CM.conj(k) * terminalB.i;
      annotation (
        Documentation(info = "<html>
    <p>A transformer can be modeled as a dedicated Pi-network. A diagram of the model is shown in the figure.</p>  
    <p>  </p>      
    <figure> <img src=\"modelica://PowerSysPro/Resources/Images/PiNetworkTra.png\"></figure>
</body></html>"));
    end myTransformer;

    model myVariableTransformer "Transformer with fixed voltage ratio and tape changer"
      extends PartialModels.myPartialTransformer;
    public
      parameter Types.myVoltage setPointU = 20400 "Setting value for U" annotation (
        Dialog(tab = "TapeChanger"));
      parameter Types.myPerUnit VoltRange = 0.14 "Range of possible voltage variation (0.12, 0.14, 0.15, 0.16, or 0.17)" annotation (
        Dialog(tab = "TapeChanger"));
      parameter Types.myPerUnit rangeU = 1.25 "Acceptable range of values for U" annotation (
        Dialog(tab = "TapeChanger"));
      parameter Integer Ntap = 17 "Number of tap positions (17, 21, or 23)" annotation (
        Dialog(tab = "TapeChanger"));
      parameter Types.myVoltage deltaTapU = VoltRange * 20000 / ((Ntap - 1) / 2) "Voltage variation between two consecutive taps: +-VoltRange*20000/((Ntap-1)/2)" annotation (
        Dialog(tab = "TapeChanger"));
      parameter Integer startTap = integer((Ntap + 1) / 2) "Starting tap position" annotation (
        Dialog(tab = "TapeChanger"));
      parameter Types.myTime firstTapChangeTimeout = 60 "Time lag before changing the first tap" annotation (
        Dialog(tab = "TapeChanger"));
      parameter Types.myTime nextTapChangeTimeout = 10 "Time lag before changing the following taps" annotation (
        Dialog(tab = "TapeChanger"));
      Regulations.myTapChanger tapChanger(setPointU = setPointU, VoltRange = VoltRange, rangeU = rangeU, Ntap = Ntap, deltaTapU = deltaTapU, startTap = startTap, firstTapChangeTimeout = firstTapChangeTimeout, nextTapChangeTimeout = nextTapChangeTimeout);
    protected
      parameter Real deltaTapRatio = VoltRange / ((Ntap - 1) / 2) "Ratio variation between two consecutive taps";
      Types.myComplexPerUnit k "Complex variable transformer ratio";
      parameter Types.myComplexImpedance Z = Complex(R, X) "Series impedance of phase conductor";
      parameter Types.myComplexAdmittance Y = Complex(G, B) "Shunt admittance at primary side";
    equation
      terminalA.i + terminalB.i * CM.conj(k) = Y * terminalA.v;
      terminalA.v - terminalB.v / k = -Z * CM.conj(k) * terminalB.i;
      tapChanger.U = UB;
      k = tapChanger.varRatio * CM.fromPolar(UNomB / UNomA, theta);
      annotation (
        Documentation(info = "<html>
    <p>A transformer can be modeled as a dedicated Pi-network. A diagram of the model is shown in the figure.</p>    
    <p>  </p>
    <figure> <img src=\"modelica://PowerSysPro/Resources/Images/PiNetworkTra.png\"></figure>
    <p>The model describes a transformer with fixed transformer ratio with Ya=0, Yb=G+jB and k = UNomB/UNomA*exp(j*theta).</p>
</body></html>"),
        Icon(graphics={  Text(extent = {{-80, 34}, {82, -34}}, lineColor = {28, 108, 200}, textString = "~", textStyle = {TextStyle.Bold})}));
    end myVariableTransformer;

    model myBreaker "Perfect breaker"
      extends Icons.myBreaker;
      extends Icons.myTwoPortsAC;
      Interfaces.myAcausalTerminal terminalA(i(re(start = 0), im(start = 0))) "Terminal A of the node" annotation (
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Interfaces.myAcausalTerminal terminalB "Terminal B of the node" annotation (
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.BooleanInput BrkOpen(start = false) "Breaker start position is false (closed)" annotation (
        Placement(visible = true, transformation(origin = {0, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Types.myCurrent IA = CM.abs(terminalA.i) "Current flowing port A";
      Types.myVoltage UA = sqrt(3) * CM.abs(terminalA.v) "Voltage at port A";
      Types.myCurrent IB = CM.abs(terminalB.i) "Current flowing port B";
      Types.myVoltage UB = sqrt(3) * CM.abs(terminalB.v) "Voltage at port B";
      Types.myApparentPower S = sqrt(3) * IA * UA "Apparent power flowing port A";
    equation
      if BrkOpen then
        terminalA.i = Complex(0);
        terminalB.i = Complex(0);
      else
        terminalA.v = terminalB.v;
        terminalA.i + terminalB.i = Complex(0);
      end if;
      annotation (
        Icon(graphics={  Line(points = {{-90, 0}, {-40, 0}}, color = {0, 0, 0}, thickness = 0.5), Line(points = {{90, 0}, {40, 0}}, color = {0, 0, 0}, thickness = 0.5), Line(points = DynamicSelect({{-40, 0}, {20, 40}}, if not BrkOpen then {{-40, 0}, {20, 40}} else {{-40, 0}, {40, 0}}), color = {0, 0, 0}, thickness = 0.5)}),
        Documentation(info = "<html>
     <p>This node prescribes a perfect breaker.</p>
    </html>"));
    end myBreaker;

    model myGround "Ground"
      extends Icons.myGround;
      Interfaces.myAcausalTerminal terminal "Terminal of the ground" annotation (
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Types.myCurrent I = CM.abs(terminal.i) "Current flowing the ground";
    equation
      terminal.v = Complex(0);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false)),
        Documentation(info = "<html>
     <p>This node prescribes a connection to the ground where voltage is null.</p>
    </html>"));
    end myGround;

    package PartialModels "Partial models to be inherited"
      extends Icons.myBasesPackage;

      partial model myPartialLoad "Partial model for loads"
        extends Icons.myLoad;
        extends Icons.myOnePortAC;
        Interfaces.myAcausalTerminal terminal(v(re(start = UNom / sqrt(3), nominal = UNom / sqrt(3)), im(start = 0))) "Terminal of the node" annotation (
          Placement(visible = true, transformation(origin = {-0.4, 0.2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-0.4, 0.2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      public
        parameter Types.myVoltage UNom "Reference voltage of the load";
        parameter Types.myPerUnit lowVoltage(min = 0, max = 1) = if UNom <= 1000 then 0.9 else 0.95 "Lower percentage limit for acceptable voltage";
        parameter Types.myPerUnit highVoltage(min = 1) = if UNom <= 1000 then 1.1 else 1.05 "Higher percentage limit for acceptable voltage";
        parameter Types.myVoltage minU = lowVoltage * UNom "Minimum acceptable value for U";
        parameter Types.myVoltage maxU = highVoltage * UNom "Maximum acceptable value for U";
        Types.myCurrent I = CM.abs(terminal.i) "Current flowing the load";
        Types.myVoltage U = sqrt(3) * CM.abs(terminal.v) "Voltage at the load";
        Types.myApparentPower S = sqrt(3) * I * U "Apparent power flowing the load";
      equation
        assert(not U < minU, ">>> Voltage is below the minimum acceptable value for " + getInstanceName(), AssertionLevel.warning);
        assert(not U > maxU, ">>> Voltage is above the maximum acceptable value for " + getInstanceName(), AssertionLevel.warning);
        annotation (
          Icon(coordinateSystem(grid = {0.1, 0.1}, initialScale = 0.1)),
          Documentation(info = "<html>
    <p>This model is a common basis for constant and variable loads.</p>
    </html>"));
      end myPartialLoad;

      partial model myPartialTransformer "Partial model for transformers"
        extends Icons.myTransformer;
        extends Icons.myTwoPortsAC;
        Interfaces.myAcausalTerminal terminalA(i(re(start = 0), im(start = 0))) "Terminal A of the node" annotation (
          Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Interfaces.myAcausalTerminal terminalB(v(re(start = UNomB / sqrt(3), nominal = UNomB / sqrt(3)), im(start = 0))) "Terminal B of the node" annotation (
          Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      public
        parameter Types.myVoltage UNomA "Reference voltage at primary side";
        parameter Types.myVoltage UNomB "Reference voltage at secondary side";
        parameter Types.myAngle theta = 0 "Fixed phase shift";
        parameter Types.myResistance R "Series resistance at primary side";
        parameter Types.myReactance X = 0 "Series reactance at primary side";
        parameter Types.myConductance G = 0 "Shunt conductance at primary side";
        parameter Types.mySusceptance B = 0 "Shunt susceptance at primary side";
        parameter Types.myApparentPowerMVA SNom = 0 "Nominal power of the transformer. 0 means no control";
        Types.myCurrent IA = CM.abs(terminalA.i) "Current flowing port A";
        Types.myVoltage UA = sqrt(3) * CM.abs(terminalA.v) "Voltage at port A";
        Types.myCurrent IB = CM.abs(terminalB.i) "Current flowing port B";
        Types.myVoltage UB = sqrt(3) * CM.abs(terminalB.v) "Voltage at port B";
        Types.myApparentPower S = sqrt(3) * IA * UA "Apparent power flowing port A";
      equation
        if SNom > 0 then
          assert(S <= 1e3 * SNom, ">>> Apparent power flowing the transformer is exceeding the nominal power for " + getInstanceName(), AssertionLevel.warning);
        end if;
        annotation (
          Documentation(info = "<html>
    <p>A transformer can be modeled as a dedicated Pi-network. A diagram of the model is shown in the figure.</p>  
    <p>  </p>      
    <figure> <img src=\"modelica://PowerSysPro/Resources/Images/PiNetworkTra.png\"></figure>
    <p>The model describes a transformer with fixed transformer ratio with Ya=0, Yb=G+jB and k = UNomB/UNomA*exp(j*theta).</p>
</body></html>"));
      end myPartialTransformer;

      model myFault "Modeling a fault during a time interval"
        Interfaces.myAcausalTerminal terminal "Port terminal" annotation (
          Placement(visible = true, transformation(origin = {-1.42109e-14, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-1.42109e-14, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        parameter Types.myResistance R = 0 "Series resistance to ground during fault";
        parameter Types.myReactance X = 0 "Series reactance to ground during fault";
        parameter Types.myTime startTime "Start time of the fault";
        parameter Types.myTime stopTime "End time of the fault";
        discrete Complex Y(re(start = 0), im(start = 0)) "Shunt admittance";
        Boolean fault(start = false) "true means the fault is active";
      algorithm
        // Fault variable
        when time > startTime then
          fault := true;
        end when;
        when time > stopTime then
          fault := false;
        end when;
        // Shunt admittance
        when pre(fault) then
          Y := 1 / Complex(R, X);
        end when;
        when not pre(fault) then
          Y := Complex(0);
        end when;
      equation
        terminal.i = Y * terminal.v;
        annotation (
          Icon(coordinateSystem(grid = {0.1, 0.1}), graphics={  Text(extent = {{-102.8, -57.6}, {103.5, -86.4}}, lineColor = {238, 46, 47}, fillColor = {238, 46, 47},
                  fillPattern =                                                                                                                                                      FillPattern.Solid, textString = "fault", textStyle = {TextStyle.Bold})}));
      end myFault;
    end PartialModels;
    annotation (
      Documentation(info = "<html><head></head><body>
    <p>The components in this package can be used to build load flow models with a minimum number of equations.</p>
    <p>The goal is to provide an easy way to calculate load flow values for Distribution networks in a Modelica environment.</p>
</body></html>"),
      Icon(graphics={  Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
              fillPattern =                                                                              FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25.0), Rectangle(lineColor = {128, 128, 128}, extent = {{-100, -100}, {100, 100}}, radius = 25.0), Text(extent = {{-78, 54}, {72, -58}}, lineColor = {28, 108, 200}, fontName = "Segoe Print", textString = "C")}));
  end Components;

  package Regulations "Regulations and controls"
    model myTapChanger "Academic tap changer regulation"
      extends Icons.myRegulation;
      // Automatic tap changer based on a setting value for U to be maintained in an acceptable value range
      // Assertions are checked about min / max tap position
      // NB: this model is simplified and tap changer defaults are not modelled
    public
      parameter Types.myVoltage setPointU = 20400 "Setting value for U, usually between 20.4 and 20.8 kV";
      parameter Types.myPerUnit VoltRange = 0.14 "Range of possible voltage variation (0.12, 0.14, 0.15, 0.16, or 0.17)";
      parameter Types.myPerUnit rangeU = 1.25 "Acceptable range of values for U around setting point";
      parameter Integer Ntap = 17 "Number of tap positions (17, 21, or 23)";
      parameter Types.myVoltage deltaTapU = VoltRange * 20000 / ((Ntap - 1) / 2) "Voltage variation between two consecutive taps: +-VoltRange*20000/((Ntap-1)/2)";
      parameter Integer startTap = integer((Ntap + 1) / 2) "Starting tap position";
      parameter Types.myTime firstTapChangeTimeout = 60 "Time lag before changing the first tap";
      parameter Types.myTime nextTapChangeTimeout = 10 "Time lag before changing the following taps";
      Integer tap(start = startTap) "Current tap position";
      Modelica.Blocks.Interfaces.RealInput U(unit = "V", displayUnit = "V") annotation (
        Placement(transformation(extent = {{-120, 30}, {-80, 70}}), iconTransformation(extent = {{-120, 22}, {-100, 42}})));
      Modelica.Blocks.Interfaces.RealOutput varRatio "Variable transformer ratio" annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 180, origin = {-100, -50}), iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-110, -30})));
    protected
      parameter Types.myVoltage underMinU = (100 - rangeU) * setPointU / 100 "Minimum acceptable value for U";
      parameter Types.myVoltage aboveMaxU = (100 + rangeU) * setPointU / 100 "Maximum acceptable value for U";
      parameter Integer tapMax = Ntap "Max tap position";
      parameter Integer tapMin = 1 "Min tap position";
      parameter Types.myPerUnit deltaTapRatio = VoltRange / (Ntap - 1) "Ratio variation between two consecutive taps: +-VoltRange/((Ntap-1)/2)";
      constant Types.myTime infTime = ModelicaServices.Machine.inf "Infinite time";
      Types.myTime abnormalUTime "Time when the signal U went out its normal range";
      Boolean normalU(start = false) "U is in the normal range [underMinU, aboveMaxU]";
      Boolean decreaseEvent(start = false) "Decreasing event to be confirmed";
      Boolean increaseEvent(start = false) "Increasing event to be confirmed";
      discrete Types.myTime furtherTapChangeTimeout(start = nextTapChangeTimeout) "Time lag between the first change and the following tap";
    algorithm
      normalU := U >= underMinU and U <= aboveMaxU;
      increaseEvent := U < underMinU;
      decreaseEvent := U > aboveMaxU;
      when U < underMinU or U > aboveMaxU then
        abnormalUTime := time;
      elsewhen not (U < underMinU or U > aboveMaxU) then
        abnormalUTime := infTime;
      end when;
      when not normalU and time > abnormalUTime + firstTapChangeTimeout then
        furtherTapChangeTimeout := nextTapChangeTimeout;
        tap := if decreaseEvent then if pre(tap) > tapMin then pre(tap) - 1 else pre(tap) else if pre(tap) < tapMax then pre(tap) + 1 else pre(tap);
      elsewhen not normalU and time > abnormalUTime + firstTapChangeTimeout + pre(furtherTapChangeTimeout) then
        furtherTapChangeTimeout := pre(furtherTapChangeTimeout) + nextTapChangeTimeout;
        tap := if decreaseEvent then if pre(tap) > tapMin then pre(tap) - 1 else pre(tap) else if pre(tap) < tapMax then pre(tap) + 1 else pre(tap);
      end when;
      varRatio := 1 + (pre(tap) - startTap) * deltaTapRatio;
    initial equation
      assert(VoltRange / Ntap < rangeU / 100, "The acceptable range of values for U is too small compared to the voltage variation between two taps for " + getInstanceName(), AssertionLevel.warning);
    equation
      assert(tap < tapMax, ">>> Maximum tap position reached for " + getInstanceName(), AssertionLevel.warning);
      assert(tap > tapMin, ">>> Minimum tap position reached for " + getInstanceName(), AssertionLevel.warning);
      annotation (
        Icon(graphics={  Text(origin = {-140.43, 18.67}, lineColor = {28, 108, 200}, extent = {{-53.57, 7.33}, {331.14, -14.66}}, textStyle = {TextStyle.Bold}, textString = "RegU")}),
        Documentation(info = "<html>
            <p>This regulation is an academic example of voltage regulation. Next figure roughly illustrates it:</p>
            <p>  </p>
            <figure> <img src=\"modelica://PowerSysPro/Resources/Images/regult.png\"></figure>
</html>"));
    end myTapChanger;

    model myQfU "Academic Q=f(U) regulation"
      extends Icons.myRegulation;
      Modelica.Blocks.Interfaces.RealInput U annotation (
        Placement(transformation(extent = {{-120, 22}, {-100, 42}}), iconTransformation(extent = {{-120, 22}, {-100, 42}})));
      Modelica.Blocks.Interfaces.RealOutput Q = hold(Qref) annotation (
        Placement(transformation(extent = {{-116, -40}, {-96, -20}}), iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-110, -30})));
      parameter Types.myActivePower Pracc_inj;
      parameter Types.myReactivePower Qmin = -0.35 * Pracc_inj;
      parameter Types.myReactivePower Qmax = 0.4 * Pracc_inj;
      parameter Types.myVoltage UNom;
      Types.myVoltage Umes[10];
      Types.myVoltage Umoy;
      Types.myReactivePower Qref(start = 0);
    equation
      when Clock(1) then
        for i in 0:8 loop
          Umes[10 - i] = Umes[10 - i - 1];
        end for;
        Umes[1] = sample(U);
        Umoy = sum(Umes) / 10;
        if sample(time) < 10 then
          Qref = 0;
        elseif Umoy < 0.96 * UNom then
          Qref = Qmax;
        elseif Umoy > 0.96 * UNom and Umoy < 0.9725 * UNom then
          Qref = -(Umoy - 0.9725 * UNom) / (0.0125 * UNom) * Qmax;
        elseif Umoy > 1.0375 * UNom and Umoy < 1.05 * UNom then
          Qref = (Umoy - 1.0375 * UNom) / (0.0125 * UNom) * Qmin;
        elseif Umoy > 1.05 * UNom then
          Qref = Qmin;
        else
          Qref = 0;
        end if;
      end when;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false), graphics={  Text(origin = {-136.43, 18.67}, lineColor = {28, 108, 200}, extent = {{-53.57, 7.33}, {331.14, -14.66}}, textStyle = {TextStyle.Bold}, textString = "Q=f(U)")}),
        Diagram(coordinateSystem(preserveAspectRatio = false)),
        Documentation(info = "<html>
    <p>This regulation is derived from document Enedis-NOI-RES_60E published by ENEDIS. A diagram of the Q=f(U) law is shown in the next figure:</p>
    <p>  </p>
    <figure> <img src=\"modelica://PowerSysPro/Resources/Images/Q=f(U).png\"></figure>
</html>"));
    end myQfU;
    annotation (
      Icon(graphics={  Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
              fillPattern =                                                                              FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25.0), Rectangle(lineColor = {128, 128, 128}, extent = {{-100, -100}, {100, 100}}, radius = 25.0), Text(extent = {{-132, 60}, {152, -58}}, lineColor = {28, 108, 200}, fontName = "Segoe Print", textString = "R")}));
  end Regulations;

  package Buses "Bus for causal connections"
    model myCausalBusVInput "Causal bus with voltage as input"
      extends Icons.myBus;
      Interfaces.myAcausalTerminal terminal annotation (
        Placement(visible = true, transformation(origin = {-1.42109e-14, 98}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput v_re(final unit = "V", displayUnit = "V") annotation (
        Placement(transformation(extent = {{-120, 44}, {-100, 64}}), iconTransformation(extent = {{-8, 8}, {8, -8}}, rotation = 0, origin = {-18, -40})));
      Modelica.Blocks.Interfaces.RealInput v_im(final unit = "V", displayUnit = "V") annotation (
        Placement(transformation(extent = {{-120, 16}, {-100, 36}}), iconTransformation(extent = {{-8, -8}, {8, 8}}, rotation = 0, origin = {-18, -80})));
      Modelica.Blocks.Interfaces.RealOutput i_re(final unit = "A", displayUnit = "A") annotation (
        Placement(transformation(extent = {{-110, -22}, {-90, -2}}), iconTransformation(extent = {{-8, -8}, {8, 8}}, rotation = 180, origin = {-18, 80})));
      Modelica.Blocks.Interfaces.RealOutput i_im(final unit = "A", displayUnit = "A") annotation (
        Placement(transformation(extent = {{-110, -52}, {-90, -32}}), iconTransformation(extent = {{-8, -8}, {8, 8}}, rotation = 180, origin = {-18, 40})));
    equation
      v_re = terminal.v.re;
      v_im = terminal.v.im;
      i_re = terminal.i.re;
      i_im = terminal.i.im;
      annotation (
        Documentation(info = "<html>
    <p> Causal input connector, with complex voltage as input and complex current as output.</p>    
    </html>"),
        Icon(graphics={  Text(origin = {73, 97}, lineColor = {0, 0, 255}, extent = {{-131, 11}, {131, -11}}, textString = "%name"), Text(extent = {{-10, -12}, {72, -52}}, lineColor = {0, 0, 0}, textStyle = {TextStyle.Bold}, textString = "~")}));
    end myCausalBusVInput;

    model myCausalBusVOutput "Causal bus with voltage as output"
      extends Icons.myBus;
      Interfaces.myAcausalTerminal terminal annotation (
        Placement(visible = true, transformation(origin = {-1.42109e-14, 98}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput v_re(final unit = "V", displayUnit = "V") annotation (
        Placement(transformation(extent = {{-110, 44}, {-90, 64}}), iconTransformation(extent = {{12, -32}, {28, -48}})));
      Modelica.Blocks.Interfaces.RealOutput v_im(final unit = "V", displayUnit = "V") annotation (
        Placement(transformation(extent = {{-110, 16}, {-90, 36}}), iconTransformation(extent = {{12, -88}, {28, -72}})));
      Modelica.Blocks.Interfaces.RealInput i_re(final unit = "A", displayUnit = "A") annotation (
        Placement(transformation(extent = {{-120, -22}, {-100, -2}}), iconTransformation(extent = {{-8, -8}, {8, 8}}, rotation = 180, origin = {20, 80})));
      Modelica.Blocks.Interfaces.RealInput i_im(final unit = "A", displayUnit = "A") annotation (
        Placement(transformation(extent = {{-120, -52}, {-100, -32}}), iconTransformation(extent = {{-8, -8}, {8, 8}}, rotation = 180, origin = {20, 40})));
    equation
      v_re = terminal.v.re;
      v_im = terminal.v.im;
      i_re + terminal.i.re = 0;
      i_im + terminal.i.im = 0;
      annotation (
        Documentation(info = "<html>
    <p> Causal output connector, with complex voltage as output and complex current as input.</p>
    </html>"),
        Icon(graphics={  Text(origin = {-79, 97}, lineColor = {0, 0, 255}, extent = {{-131, 11}, {131, -11}}, textString = "%name"), Text(extent = {{-70, -10}, {12, -50}}, lineColor = {0, 0, 0}, textStyle = {TextStyle.Bold}, textString = "~")}));
    end myCausalBusVOutput;
    annotation (
      Icon(coordinateSystem(grid = {0.1, 0.1}), graphics={  Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
              fillPattern =                                                                                                                   FillPattern.HorizontalCylinder, extent = {{-100, -101.1}, {100, 98.9}}, radius = 25.0), Rectangle(lineColor = {128, 128, 128}, extent = {{-100, -101.1}, {100, 98.9}}, radius = 25.0), Text(extent = {{-78, 52.9}, {72, -59.1}}, lineColor = {28, 108, 200}, fontName = "Segoe Print", textString = "B")}),
      Diagram(coordinateSystem(extent = {{-200, -100}, {200, 100}})),
      Documentation(info = "<html>
    <p>These buses are causal connectors.</p>
    <p>Causal connectors are to be used when the model must be split before exporting as FMUs.</p>
</body></html>"));
  end Buses;

  package Sensors "Ideal Sensors"
    extends Icons.mySensorsPackage;

    model VoltMeter "Ideal voltmeter"
      extends Icons.mySensor;
      Interfaces.myAcausalTerminal terminal "Terminal of the sensor" annotation (
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Umes(unit = "V", displayUnit = "kV") "Measured voltage" annotation (
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-80, 90}), iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {0, 90})));
    equation
      Umes = sqrt(3) * CM.abs(terminal.v);
      terminal.i = Complex(0);
      annotation (
        Documentation(info = "<html>
    <p>This equipement measures the voltage at the connected terminal.</p>
    </html>"),
        Icon(coordinateSystem(preserveAspectRatio = false, initialScale = 0.1), graphics={  Text(extent = {{-178, -4}, {102, -38}}, lineColor = {0, 0, 255},
                lineThickness =                                                                                                                                              0.5, fillColor = {255, 255, 255},
                fillPattern =                                                                                                                                                                                                FillPattern.Solid, textString = "V")}));
    end VoltMeter;

    model AmMeter "Ideal ammeter"
      extends Icons.mySensor;
      Interfaces.myAcausalTerminal terminalA "Terminal A of the node" annotation (
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Interfaces.myAcausalTerminal terminalB "Terminal B of the node" annotation (
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Imes(unit = "A", displayUnit = "A") "Measured flowing current at port A" annotation (
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-40, 90}), iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {0, 90})));
    equation
      Imes = CM.abs(terminalA.i);
      terminalB = terminalA;
      annotation (
        Documentation(info = "<html>
    <p>This equipement measures the current flowing through the two connected terminals.</p>
    </html>"),
        Icon(coordinateSystem(preserveAspectRatio = false, initialScale = 0.1), graphics={  Text(extent = {{-178, -4}, {102, -38}}, lineColor = {0, 0, 255},
                lineThickness =                                                                                                                                              0.5, fillColor = {255, 255, 255},
                fillPattern =                                                                                                                                                                                                FillPattern.Solid, textString = "A"), Polygon(origin = {40, -83}, fillColor = {70, 70, 70}, pattern = LinePattern.None,
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{0, 19}, {40, 0}, {0, -21}, {0, 19}}), Rectangle(fillColor = {70, 70, 70}, pattern = LinePattern.None,
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{-70, -78}, {40, -88}})}));
    end AmMeter;

    model WattMeter "Versatile sensor"
      extends Icons.mySensor;
      Interfaces.myAcausalTerminal terminalA "Terminal A of the node" annotation (
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Interfaces.myAcausalTerminal terminalB "Terminal B of the node" annotation (
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Umes(unit = "V", displayUnit = "kV") "Measured voltage at port A" annotation (
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-80, 90}), iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-40, 90})));
      Modelica.Blocks.Interfaces.RealOutput Imes(unit = "A", displayUnit = "A") "Measured flowing current at port A" annotation (
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {0, 90}), iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {40, 90})));
      Modelica.Blocks.Interfaces.RealOutput Pmes(unit = "W", displayUnit = "kW") "Measured active flowing power at port A";
      Modelica.Blocks.Interfaces.RealOutput Qmes(unit = "var", displayUnit = "kvar") "Measured reactive flowing power at port A";
      Modelica.Blocks.Interfaces.RealOutput Smes(unit = "kVA", displayUnit = "kVA") "Measured apparent flowing power at port A";
    equation
      Umes = sqrt(3) * CM.abs(terminalA.v);
      Imes = CM.abs(terminalA.i);
      Smes = sqrt(3) * Imes * Umes;
      Complex(Pmes, Qmes) = 3 * terminalA.v * CM.conj(terminalA.i);
      terminalB = terminalA;
      annotation (
        Documentation(info = "<html>
    <p>This equipement measures the active, reactive and apparent power flowing through the two connected terminals.</p>
    </html>"),
        Icon(coordinateSystem(preserveAspectRatio = false, initialScale = 0.1), graphics={  Text(extent = {{-178, -4}, {102, -38}}, lineColor = {0, 0, 255},
                lineThickness =                                                                                                                                              0.5, fillColor = {255, 255, 255},
                fillPattern =                                                                                                                                                                                                FillPattern.Solid, textString = "W"), Polygon(origin = {40, -83}, fillColor = {70, 70, 70}, pattern = LinePattern.None,
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{0, 19}, {40, 0}, {0, -21}, {0, 19}}), Rectangle(fillColor = {70, 70, 70}, pattern = LinePattern.None,
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{-70, -78}, {40, -88}})}));
    end WattMeter;

    model DiffMeter "Drop sensor"
      extends Icons.mySensor;
      Modelica.Blocks.Interfaces.RealOutput deltaU(unit = "V", displayUnit = "kV") "Measured voltage drop from port A to port B";
      Modelica.Blocks.Interfaces.RealOutput deltaI(unit = "A", displayUnit = "A") "Measured current drop from port A to port B";
      Modelica.Blocks.Interfaces.RealOutput deltaS(unit = "kVA", displayUnit = "kVA") "Measured apparent power drop from port A to port B";
      Modelica.Blocks.Interfaces.RealInput UA(unit = "V", displayUnit = "kV") "Voltage at port A" annotation (
        Placement(transformation(extent = {{-132, 14}, {-100, 46}}), iconTransformation(extent = {{-132, 14}, {-100, 46}})));
      Modelica.Blocks.Interfaces.RealInput IA(unit = "A", displayUnit = "A") "Current flowing into port A" annotation (
        Placement(transformation(extent = {{-132, -48}, {-100, -16}}), iconTransformation(extent = {{-132, -48}, {-100, -16}})));
      Modelica.Blocks.Interfaces.RealInput UB(unit = "V", displayUnit = "kV") "Voltage at port B" annotation (
        Placement(transformation(extent = {{-16, -16}, {16, 16}}, rotation = 180, origin = {116, 30}), iconTransformation(extent = {{-16, -16}, {16, 16}}, rotation = 180, origin = {116, 30})));
      Modelica.Blocks.Interfaces.RealInput IB(unit = "A", displayUnit = "A") "Current flowing into port B" annotation (
        Placement(transformation(extent = {{-16, -16}, {16, 16}}, rotation = 180, origin = {116, -30}), iconTransformation(extent = {{-16, -16}, {16, 16}}, rotation = 180, origin = {116, -30})));
    equation
      deltaU = UA - UB;
      deltaI = IA - IB;
      deltaS = sqrt(3) * (IA * UA - IB * UB);
      annotation (
        Documentation(info = "<html>
    <p>This equipement measures the difference of voltage, current, and apparent power between its two connected terminals.</p>
    </html>"),
        Icon(graphics={  Polygon(origin = {40, -83}, fillColor = {70, 70, 70}, pattern = LinePattern.None,
                fillPattern =                                                                                            FillPattern.Solid, points = {{0, 19}, {40, 0}, {0, -21}, {0, 19}}), Rectangle(fillColor = {70, 70, 70}, pattern = LinePattern.None,
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{-70, -78}, {40, -88}}), Text(extent = {{-140, 128}, {140, 94}}, lineColor = {0, 0, 255},
                lineThickness =                                                                                                                                                                                                        0.5, fillColor = {255, 255, 255},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, textString = "delta")}));
    end DiffMeter;
    annotation (
      Icon(coordinateSystem(grid = {0.1, 0.1}), graphics={  Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
              fillPattern =                                                                                                                   FillPattern.HorizontalCylinder, extent = {{-100, -101.1}, {100, 98.9}}, radius = 25.0), Rectangle(lineColor = {128, 128, 128}, extent = {{-100, -101.1}, {100, 98.9}}, radius = 25.0), Text(extent = {{-78, 52.9}, {72, -59.1}}, lineColor = {28, 108, 200}, fontName = "Segoe Print", textString = "S")}),
      Diagram(coordinateSystem(extent = {{-200, -100}, {200, 100}})),
      Documentation(info = "<html>
    <p>Ideal sensors are proposed to measure electric states.</p>
    <p>One of them measures the drop of electric states between two connected terminals.</p>
</body></html>"));
  end Sensors;

  package Interfaces "Interfaces for connecting the components"
    extends Icons.myInterfacesPackage;

    connector myAcausalTerminal "Non-causal terminal for phasor-based AC connections"
      //Dymola issue
      //Modelica.Units.SI.ComplexVoltage v "Phase-to-ground voltage phasor";
      //flow Modelica.Units.SI.ComplexCurrent i "Current phasor";
      Types.myComplexVoltage v "Phase-to-ground voltage phasor";
      flow Types.myComplexCurrent i "Current phasor";
      annotation (
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 1, grid = {2, 2}), graphics={  Rectangle(origin = {92, 3}, fillColor = {85, 170, 255},
                fillPattern =                                                                                                                                                                                      FillPattern.Solid, extent = {{-192, 97}, {8, -103}})}),
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 1, grid = {2, 2})),
        Documentation(info = "<html><head></head><body>
<p>
The myAcausalTerminal connector represents an AC terminal with voltage and flow current.
</body></html>"));
    end myAcausalTerminal;
    annotation (
      Documentation(info = "<html>
    <p>Each interface in this package is composed of a complex voltage v (phase-to-ground voltage phasor) and a flow complex current i (line current phasor).</p>
</body></html>"));
  end Interfaces;

  package Functions "Functions and constants"
    extends Icons.myPackage;
    final constant Real pi= 2 * Functions.asin(1.0) "Pi number";

   function abs "Absolute value of complex number"
      extends Icons.myFunction;
      input Complex c "Complex number";
      output Real result "= abs(c)";

   algorithm
      result := (c.re ^ 2 + c.im ^ 2) ^ 0.5;

   end abs;

   function fromPolar "Complex from polar representation"
      extends Icons.myFunction;
      input Real len "abs of complex";
      input Types.myAngle phi "arg of complex";
      output Complex c "= len * cos(phi) + j * len * sin(phi)";
   algorithm
      c := Complex(len * Functions.cos(phi), len * Functions.sin(phi));

   end fromPolar;

   function conj "Conjugate of complex number"
      extends Icons.myFunction;
      input Complex c1 "Complex number";
      output Complex c2 "= c1.re - j*c1.im";
   algorithm
      c2 := Complex(c1.re, -c1.im);

   end conj;

  function cos "Cosine"
    extends Icons.myFunction;
    input Types.myAngle u "Independent variable";
    output Real y "Dependent variable y=cos(u)";

  external "builtin" y = cos(u);

  end cos;

  function sin "Sine"
    extends Icons.myFunction;
    input Types.myAngle u "Independent variable";
    output Real y "Dependent variable y=sin(u)";

  external "builtin" y = sin(u);

  end sin;

  function asin "Inverse sine (-1 <= u <= 1)"
    extends Modelica.Math.Icons.AxisCenter;
    input Real u "Independent variable";
    output Modelica.Units.SI.Angle y "Dependent variable y=asin(u)";

  external "builtin" y = asin(u);


  end asin;
    annotation (
      Icon(graphics={  Text(extent = {{-72, 58.9}, {78, -53.1}}, lineColor = {28, 108, 200}, fontName = "Segoe Print", textString = "F")}),
      Documentation(info = "<html>
    <p>Rewrite MSL functions for portability needs.</p>
</body></html>"));
  end Functions;

  package Types "Domain-specific type definitions"
    extends Icons.myTypesPackage;
    type myVoltage = Real(nominal = 1e3, unit = "V", displayUnit = "kV");
    type myCurrent = Real(nominal = 1e-3, unit = "A", displayUnit = "A");
    type myActivePower = Real(unit = "W", displayUnit = "kW");
    type myReactivePower = Real(unit = "var", displayUnit = "kvar");
    type myApparentPower = Real(unit = "VA", displayUnit = "kVA");
    type myApparentPowerMVA = Real(unit = "kVA", displayUnit = "MVA");
    type myResistance = Real(unit = "Ohm", displayUnit = "Ohm");
    type myReactance = Real(unit = "Ohm", displayUnit = "Ohm");
    type myConductance = Real(unit = "S", displayUnit = "S");
    type mySusceptance = Real(unit = "S", displayUnit = "S");
    type myAngle = Real(unit = "rad", displayUnit = "deg");
    type myPerUnit = Real(unit = "1");
    type myTime = Real(unit = "s", displayUnit = "s");
    operator record myComplexVoltage =
      Complex(redeclare myVoltage re "Imaginary part of complex voltage",
              redeclare myVoltage im "Real part of complex voltage")
      "Complex voltage";
    operator record myComplexCurrent =
      Complex(redeclare myCurrent re "Real part of complex current",
              redeclare myCurrent im "Imaginary part of complex current")
      "Complex current";
    operator record myComplexAdmittance =
      Complex(redeclare myConductance re "Real part of complex admittance (conductance)",
              redeclare mySusceptance im "Imaginary part of complex admittance (susceptance)")
      "Complex admittance";
    operator record myComplexImpedance =
      Complex(redeclare myResistance re "Real part of complex impedance (resistance)",
              redeclare myReactance im "Imaginary part of complex impedance (reactance)")
      "Complex impedance";
    operator record myComplexPerUnit =
       Complex(re(unit = "1"),
               im(unit = "1"))
       "Complexe per unit";
    annotation (
      Documentation(info = "<html>
    <p>This package gathers dedicated icons for all models in this library.</p>
</body></html>"));
  end Types;

  package Icons "Icons for the library models"
    extends Icons.myIconsPackage;

    model mySource "Icon for fixed source node"
      annotation (
        Icon(graphics={  Ellipse(origin = {0, -1}, extent = {{-60, 61}, {60, -59}}, endAngle = 360), Text(origin = {0, 96}, lineColor = {0, 0, 255}, extent = {{-118, 12}, {118, -12}}, textString = "%name"), Text(origin = {-138.43, -89.33}, lineColor = {28, 108, 200}, extent = {{-53.57, 7.33}, {331.14, -14.66}}, textStyle = {TextStyle.Bold}, textString = "%UNom"), Line(origin = {6.7702, 12.33}, points = {{-51.4286, -11.4286}, {-44.9714, 8.11429}, {-40.8571, 18.9143}, {-37.2, 26.5143}, {-33.9429, 31.2}, {-30.7429, 33.7714}, {-27.5429, 34.1714}, {-24.3429, 32.3429}, {-21.0857, 28.4}, {-17.8857, 22.5143}, {-14.2286, 13.7714}, {-9.61714, 0.685714}, {0.0571429, -29.0286}, {4.17143, -40.1143}, {7.82857, -48.1143}, {11.0286, -53.2}, {14.2857, -56.2286}, {17.4857, -57.1429}, {20.6857, -55.7714}, {23.9429, -52.2857}, {27.1429, -46.8}, {30.8, -38.4}, {35.4286, -25.6}, {40, -11.4286}}, smooth = Smooth.Bezier)}, coordinateSystem(initialScale = 0.1)));
    end mySource;

    model myPV "Icon for PV node"
      annotation (
        Icon(graphics={  Ellipse(origin = {0, -1}, extent = {{-60, 61}, {60, -59}}, endAngle = 360), Text(origin = {0, 96}, lineColor = {0, 0, 255}, extent = {{-118, 12}, {118, -12}}, textString = "%name"), Text(origin = {-138.43, -89.33}, lineColor = {28, 108, 200}, extent = {{-53.57, 7.33}, {331.14, -14.66}}, textStyle = {TextStyle.Bold}, textString = "%UNom")}, coordinateSystem(initialScale = 0.1)));
    end myPV;

    model myLoad "Icon for load node"
      annotation (
        Icon(graphics={  Polygon(origin = {70, 0}, rotation = 90, fillPattern = FillPattern.Solid, points = {{-40, 30}, {40, 30}, {0, -30}, {-40, 30}}), Text(origin = {62, 67}, lineColor = {0, 0, 255}, extent = {{-124, 11}, {124, -11}}, textString = "%name"), Line(points = {{0, 0}, {40, 0}})}, coordinateSystem(initialScale = 0.1)));
    end myLoad;

    model myCapacitorBank "Icon for capacitor bank"
      annotation (
        Icon(graphics={  Polygon(origin = {70, 0}, fillPattern = FillPattern.Solid, points = {{-40, 30}, {40, 30}, {0, -30}, {-40, 30}}, rotation = 90), Rectangle(origin = {29, 0},
                fillPattern =                                                                                                                                                                      FillPattern.Solid, extent = {{-20, 1}, {20, -1}}, rotation = 270), Line(origin = {32, 0}, points = {{0, 8}, {1.07002e-15, -2}}, rotation = 270), Line(origin = {12, 0}, points = {{-4.88625e-16, 6}, {7.34788e-16, -6}}, rotation = 270), Text(origin = {62, 67}, lineColor = {0, 0, 255}, extent = {{-124, 11}, {124, -11}}, textString = "%name"), Rectangle(origin = {19, 0},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{-20, 1}, {20, -1}}, rotation = 270)}, coordinateSystem(initialScale = 0.1)));
    end myCapacitorBank;

    model myLine "Icon for line"
      annotation (
        Icon(graphics={  Rectangle(origin = {-1, -1}, fillColor = {255, 255, 255},
                fillPattern =                                                                    FillPattern.Solid, extent = {{-59, 11}, {61, -11}}), Line(origin = {-80, 0}, points = {{-20, 0}, {20, 0}}), Line(origin = {80, 0}, points = {{-20, 0}, {20, 0}}), Text(origin = {0, 64}, lineColor = {0, 0, 255}, extent = {{-118, 12}, {118, -12}}, textString = "%name"), Text(origin = {-108.435, -55.3303}, lineColor = {28, 108, 200}, extent = {{-53.5654, 7.33031}, {265.143, -16.6612}}, textStyle = {TextStyle.Bold}, textString = "%Imax")}, coordinateSystem(initialScale = 0.1)));
    end myLine;

    model myTransformer "Icon for transformer"
      annotation (
        Icon(graphics={  Ellipse(origin = {-15, -7}, extent = {{-45, 47}, {35, -33}}, endAngle = 360), Ellipse(origin = {3, -9}, extent = {{57, 49}, {-23, -31}}, endAngle = 360), Line(origin = {-80, 0}, points = {{-20, 0}, {20, 0}, {20, 0}}), Line(origin = {80, 0}, points = {{-20, 0}, {20, 0}}), Text(origin = {-1, 94}, lineColor = {0, 0, 255}, extent = {{-179, 14}, {179, -14}}, textString = "%name"), Text(origin = {-91.9974, -90.6665}, lineColor = {28, 108, 200}, extent = {{-92.004, 6.66649}, {281.997, -11.3335}}, textStyle = {TextStyle.Bold}, textString = "%SNom")}, coordinateSystem(initialScale = 0.1)));
    end myTransformer;

    model myBreaker "Icon for breaker"
      annotation (
        Icon(graphics={  Line(points = {{90, 0}, {40, 0}}, color = {0, 0, 0}, thickness = 0.5), Text(origin = {0, 68}, lineColor = {0, 0, 255}, extent = {{-118, 12}, {118, -12}}, textString = "%name"), Line(points = {{-40, 0}, {-90, 0}}, color = {0, 0, 0}, thickness = 0.5)}));
    end myBreaker;

    model myGround "Icon for ground"
      annotation (
        Icon(graphics={  Line(origin = {0, -20}, points = {{0, 20}, {0, -20}, {0, -20}}), Line(origin = {0, -40}, points = {{-40, 0}, {40, 0}, {40, 0}, {40, 0}}), Line(origin = {0, -60}, points = {{-20, 0}, {20, 0}, {20, 0}}), Line(origin = {0, -80}, points = {{-4, 0}, {4, 0}})}, coordinateSystem(initialScale = 0.1)));
    end myGround;

    model myBus "Icon for causal bus"
      annotation (
        Icon(graphics={  Rectangle(origin = {1, 25}, fillPattern = FillPattern.Solid, extent = {{-5, 75}, {5, -125}})}, coordinateSystem(initialScale = 0.1)));
    end myBus;

    model myRegulation "Icon for tape changer"
      annotation (
        Icon(coordinateSystem(initialScale = 0.2), graphics={  Rectangle(extent = {{-100, 40}, {98, -50}}, fillColor = {255, 255, 255},
                fillPattern =                                                                                                                         FillPattern.Solid, pattern = LinePattern.None), Line(points = {{-76, -18}, {-48, 24}, {-12, -50}, {20, -4}, {60, -30}, {82, 20}}, color = {0, 0, 255}, smooth = Smooth.None), Line(points = {{-88, 0}, {88, 0}}, color = {0, 0, 0}), Line(points = {{-86, 32}, {90, 32}}, color = {192, 192, 192}, thickness = 1), Line(points = {{-88, -30}, {88, -30}}, color = {192, 192, 192}, thickness = 1)}));
    end myRegulation;

    model myFault "Icon for fault"
      annotation (
        Icon(graphics={  Rectangle(origin = {-1, -1}, fillColor = {238, 46, 47},
                fillPattern =                                                                  FillPattern.Solid, extent = {{-59, 11}, {61, -11}}, lineColor = {238, 46, 47})}, coordinateSystem(initialScale = 0.1)));
    end myFault;

    model mySensor "Icon for sensor"
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false), graphics={  Ellipse(fillColor = {245, 245, 245},
                fillPattern =                                                                                              FillPattern.Solid, extent = {{-70, -68}, {70, 72}}), Line(points = {{-37.6, 15.7}, {-65.8, 25.9}}), Line(points = {{-22.9, 34.8}, {-40.2, 59.3}}), Line(points = {{0, 72}, {0, 42}}), Line(points = {{22.9, 34.8}, {40.2, 59.3}}), Line(points = {{37.6, 15.7}, {65.8, 25.9}}), Polygon(origin = {0, 2}, rotation = -17.5, fillColor = {64, 64, 64}, pattern = LinePattern.None,
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{-5.0, 0.0}, {-2.0, 60.0}, {0.0, 65.0}, {2.0, 60.0}, {5.0, 0.0}}), Ellipse(lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, extent = {{-12, -10}, {12, 14}}), Ellipse(fillColor = {64, 64, 64}, pattern = LinePattern.None,
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{-7, -5}, {7, 9}}), Text(extent = {{-29, -9}, {30, -68}}, lineColor = {0, 0, 0}, textString = "I"), Line(points = {{-70, 0}, {-100, 0}}, color = {0, 0, 255}, thickness = 0.5), Line(points = {{70, 0}, {100, 0}}, color = {0, 0, 255}, thickness = 0.5), Text(origin = {-2, -133}, lineColor = {0, 0, 255}, extent = {{-124, 11}, {124, -11}}, textString = "%name")}),
        Diagram(coordinateSystem(preserveAspectRatio = false)));
    end mySensor;

    model myTwoPortsAC "Completing two ports AC icon"
      annotation (
        Icon(coordinateSystem(initialScale = 0.1), graphics={  Text(extent = {{-144, 52}, {-52, 28}}, lineColor = {0, 0, 0}, textString = "A", textStyle = {TextStyle.Bold}), Text(extent = {{48, 52}, {160, 28}}, lineColor = {0, 0, 0}, textStyle = {TextStyle.Bold}, textString = "B"), Text(extent = {{-140, -12}, {-58, -52}}, lineColor = {0, 0, 0}, textStyle = {TextStyle.Bold}, textString = "~"), Text(extent = {{60, -10}, {142, -50}}, lineColor = {0, 0, 0}, textStyle = {TextStyle.Bold}, textString = "~")}));
    end myTwoPortsAC;

    model myOnePortAC "Completing one port AC icon"
      annotation (
        Icon(coordinateSystem(initialScale = 0.1), graphics={  Text(extent = {{-40, -12}, {42, -52}}, lineColor = {0, 0, 0}, textStyle = {TextStyle.Bold}, textString = "~")}));
    end myOnePortAC;

    partial function myFunction "Icon for functions"

      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
            Text(
              textColor={0,0,255},
              extent={{-150,105},{150,145}},
              textString="%name"),
            Ellipse(
              lineColor = {108,88,49},
              fillColor = {255,215,136},
              fillPattern = FillPattern.Solid,
              extent = {{-100,-100},{100,100}}),
            Text(
              textColor={108,88,49},
              extent={{-90.0,-90.0},{90.0,90.0}},
              textString="f")}),
    Documentation(info="<html>
<p>This icon indicates functions.</p>
</html>"));
    end myFunction;

    partial model myExample "Icon for runnable tests and examples"

      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
            Ellipse(lineColor = {75,138,73},
                    fillColor={255,255,255},
                    fillPattern = FillPattern.Solid,
                    extent = {{-100,-100},{100,100}}),
            Polygon(lineColor = {0,0,255},
                    fillColor = {75,138,73},
                    pattern = LinePattern.None,
                    fillPattern = FillPattern.Solid,
                    points = {{-36,60},{64,0},{-36,-60},{-36,60}})}), Documentation(info="<html>
<p>This icon indicates an example. The play button suggests that the example can be executed.</p>
</html>"));
    end myExample;

    partial record myRecord "Icon for records"

      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}), graphics={
            Text(
              textColor={0,0,255},
              extent={{-150,60},{150,100}},
              textString="%name"),
            Rectangle(
              origin={0.0,-25.0},
              lineColor={64,64,64},
              fillColor={255,215,136},
              fillPattern=FillPattern.Solid,
              extent={{-100.0,-75.0},{100.0,75.0}},
              radius=25.0),
            Line(
              points={{-100.0,0.0},{100.0,0.0}},
              color={64,64,64}),
            Line(
              origin={0.0,-50.0},
              points={{-100.0,0.0},{100.0,0.0}},
              color={64,64,64}),
            Line(
              origin={0.0,-25.0},
              points={{0.0,75.0},{0.0,-75.0}},
              color={64,64,64})}), Documentation(info="<html>
<p>
This icon is indicates a record.
</p>
</html>"));
    end myRecord;

    partial package myPackage "Icon for standard packages"
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248},
                fillPattern =                                                                                                                                                                  FillPattern.HorizontalCylinder, extent = {{-100.0, -100.0}, {100.0, 100.0}}, radius = 25.0), Rectangle(lineColor = {128, 128, 128}, extent = {{-100.0, -100.0}, {100.0, 100.0}}, radius = 25.0)}),
        Documentation(info = "<html>
<p>Standard package icon.</p>
</html>"));
    end myPackage;

    partial package myBasesPackage  "Icon for packages containing base classes"
      extends Icons.myPackage;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics={
            Ellipse(
              extent={{-30.0,-30.0},{30.0,30.0}},
              lineColor={128,128,128},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}),
                                Documentation(info="<html>
<p>This icon shall be used for a package/library that contains base models and classes, respectively.</p>
</html>"));
    end myBasesPackage;

    partial package mySensorsPackage "Icon for packages containing sensors"
      extends Icons.myPackage;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Ellipse(origin = {0.0, -30.0}, fillColor = {255, 255, 255}, extent = {{-90.0, -90.0}, {90.0, 90.0}}, startAngle = 20.0, endAngle = 160.0), Ellipse(origin = {0.0, -30.0}, fillColor = {128, 128, 128}, pattern = LinePattern.None,
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{-20.0, -20.0}, {20.0, 20.0}}), Line(origin = {0.0, -30.0}, points = {{0.0, 60.0}, {0.0, 90.0}}), Ellipse(origin = {-0.0, -30.0}, fillColor = {64, 64, 64}, pattern = LinePattern.None,
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{-10.0, -10.0}, {10.0, 10.0}}), Polygon(origin = {-0.0, -30.0}, rotation = -35.0, fillColor = {64, 64, 64}, pattern = LinePattern.None,
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{-7.0, 0.0}, {-3.0, 85.0}, {0.0, 90.0}, {3.0, 85.0}, {7.0, 0.0}})}),
        Documentation(info = "<html>
<p>This icon indicates a package containing sensors.</p>
</html>"));
    end mySensorsPackage;

    partial package myInterfacesPackage "Icon for packages containing interfaces"
      extends Icons.myPackage;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Polygon(origin = {20.0, 0.0}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255},
                fillPattern =                                                                                                                                                                                   FillPattern.Solid, points = {{-10.0, 70.0}, {10.0, 70.0}, {40.0, 20.0}, {80.0, 20.0}, {80.0, -20.0}, {40.0, -20.0}, {10.0, -70.0}, {-10.0, -70.0}}), Polygon(fillColor = {102, 102, 102}, pattern = LinePattern.None,
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{-100.0, 20.0}, {-60.0, 20.0}, {-30.0, 70.0}, {-10.0, 70.0}, {-10.0, -70.0}, {-30.0, -70.0}, {-60.0, -20.0}, {-100.0, -20.0}})}),
        Documentation(info = "<html>
<p>This icon indicates packages containing interfaces.</p>
</html>"));
    end myInterfacesPackage;

    partial package myTypesPackage "Icon for packages containing type definitions"
      extends Icons.myPackage;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Polygon(origin = {-12.167, -23}, fillColor = {128, 128, 128}, pattern = LinePattern.None,
                fillPattern =                                                                                                                                                                                        FillPattern.Solid, points = {{12.167, 65}, {14.167, 93}, {36.167, 89}, {24.167, 20}, {4.167, -30}, {14.167, -30}, {24.167, -30}, {24.167, -40}, {-5.833, -50}, {-15.833, -30}, {4.167, 20}, {12.167, 65}}, smooth = Smooth.Bezier), Polygon(origin = {2.7403, 1.6673}, fillColor = {128, 128, 128}, pattern = LinePattern.None,
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{49.2597, 22.3327}, {31.2597, 24.3327}, {7.2597, 18.3327}, {-26.7403, 10.3327}, {-46.7403, 14.3327}, {-48.7403, 6.3327}, {-32.7403, 0.3327}, {-6.7403, 4.3327}, {33.2597, 14.3327}, {49.2597, 14.3327}, {49.2597, 22.3327}}, smooth = Smooth.Bezier)}));
    end myTypesPackage;

    partial package myIconsPackage "Icon for packages containing icons"
      extends Icons.myPackage;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Polygon(origin = {-8.167, -17}, fillColor = {128, 128, 128}, pattern = LinePattern.None,
                fillPattern =                                                                                                                                                                                       FillPattern.Solid, points = {{-15.833, 20.0}, {-15.833, 30.0}, {14.167, 40.0}, {24.167, 20.0}, {4.167, -30.0}, {14.167, -30.0}, {24.167, -30.0}, {24.167, -40.0}, {-5.833, -50.0}, {-15.833, -30.0}, {4.167, 20.0}, {-5.833, 20.0}}, smooth = Smooth.Bezier), Ellipse(origin = {-0.5, 56.5}, fillColor = {128, 128, 128}, pattern = LinePattern.None,
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{-12.5, -12.5}, {12.5, 12.5}})}));
    end myIconsPackage;

    partial package myExamplesPackage "Icon for packages containing tests and examples"
      extends Icons.myPackage;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Polygon(origin = {8.0, 14.0}, lineColor = {78, 138, 73}, fillColor = {78, 138, 73}, pattern = LinePattern.None,
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{-58.0, 46.0}, {42.0, -14.0}, {-58.0, -74.0}, {-58.0, 46.0}})}),
        Documentation(info = "<html>
<p>This icon indicates a package that contains executable examples.</p>
</html>"));
    end myExamplesPackage;

    partial class myInformation "Icon for general information packages"
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Ellipse(lineColor = {75, 138, 73}, fillColor = {75, 138, 73}, pattern = LinePattern.None,
                fillPattern =                                                                                                                                                                                        FillPattern.Solid, extent = {{-100.0, -100.0}, {100.0, 100.0}}), Polygon(origin = {-4.167, -15.0}, fillColor = {255, 255, 255}, pattern = LinePattern.None,
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, points = {{-15.833, 20.0}, {-15.833, 30.0}, {14.167, 40.0}, {24.167, 20.0}, {4.167, -30.0}, {14.167, -30.0}, {24.167, -30.0}, {24.167, -40.0}, {-5.833, -50.0}, {-15.833, -30.0}, {4.167, 20.0}, {-5.833, 20.0}}, smooth = Smooth.Bezier), Ellipse(origin = {7.5, 56.5}, fillColor = {255, 255, 255}, pattern = LinePattern.None,
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{-12.5, -12.5}, {12.5, 12.5}})}),
        Documentation(info = "<html>
<p>This icon indicates classes containing only documentation, intended for general description of, e.g., concepts and features of a package.</p>
</html>"));
    end myInformation;

    partial class myReleaseNotes "Icon for general information"
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Polygon(points = {{-80, -100}, {-80, 100}, {0, 100}, {0, 20}, {80, 20}, {80, -100}, {-80, -100}}, fillColor = {245, 245, 245},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Polygon(points = {{0, 100}, {80, 20}, {0, 20}, {0, 100}}, fillColor = {215, 215, 215},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Line(points = {{2, -12}, {50, -12}}), Ellipse(extent = {{-56, 2}, {-28, -26}}, fillColor = {215, 215, 215},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Line(points = {{2, -60}, {50, -60}}), Ellipse(extent = {{-56, -46}, {-28, -74}}, fillColor = {215, 215, 215},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid)}),
        Documentation(info = "<html>
<p>This icon indicates release notes and the revision history of a library.</p>
</html>"));
    end myReleaseNotes;

    partial class myContact "Icon for contact information"
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Rectangle(extent = {{-100, 70}, {100, -72}}, fillColor = {235, 235, 235},
                fillPattern =                                                                                                                                                                        FillPattern.Solid), Polygon(points = {{-100, -72}, {100, -72}, {0, 20}, {-100, -72}}, fillColor = {215, 215, 215},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Polygon(points = {{22, 0}, {100, 70}, {100, -72}, {22, 0}}, fillColor = {235, 235, 235},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Polygon(points = {{-100, 70}, {100, 70}, {0, -20}, {-100, 70}}, fillColor = {241, 241, 241},
                fillPattern =                                                                                                                                                                                                        FillPattern.Solid)}),
        Documentation(info = "<html>
<p>This icon shall be used for the contact information of the library developers.</p>
</html>"));
    end myContact;
    annotation (
      Documentation(info = "<html>
    <p>This package contents dedicated icons for the basic components.</p>
    </body></html>"));
  end Icons;

  package Tests "Tests package based on single component testing"
    extends Icons.myExamplesPackage;

    model OneSource "Testing a source"
      extends Icons.myExample;
      Components.mySource src(UNom = 10000, theta = 0.5235987755983) annotation (
        Placement(visible = true, transformation(origin = {-52, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Buses.myCausalBusVOutput Bout annotation (
        Placement(transformation(extent = {{-26, -16}, {-6, 4}})));
      Modelica.Blocks.Interfaces.RealInput i_re(start = -30) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 180, origin = {100, 24})));
      Modelica.Blocks.Interfaces.RealInput i_im(start = -40) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 180, origin = {100, -8})));
    equation
      connect(i_re, Bout.i_re) annotation (
        Line(points = {{100, 24}, {22, 24}, {22, 2}, {-14, 2}}, color = {0, 0, 127}));
      connect(src.terminal, Bout.terminal) annotation (
        Line(points = {{-52, -6}, {-19, -6}}, color = {0, 0, 0}));
      connect(i_im, Bout.i_im) annotation (
        Line(points = {{100, -8}, {22, -8}, {22, -2}, {-14, -2}}, color = {0, 0, 127}));
      annotation (
        Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-108, 24}, {34, 14}}, fontSize = 12, textString = "U = 10 kV, theta = 30°"), Text(lineColor = {28, 108, 200}, extent = {{-94, -20}, {106, -60}}, fontSize = 12, textString = "current in the source is 50 A
apparent power flowing the source is 866 kVA"), Text(lineColor = {28, 108, 200}, extent = {{10, 48}, {152, 38}}, fontSize = 12, textString = "i_re = -30, i_im = -40")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSource;

    model OneLoad "Testing a load"
      extends Icons.myExample;
      Components.myLoad load(UNom = 5000, P = 4000, Q = 3000) annotation (
        Placement(visible = true, transformation(origin = {66, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Buses.myCausalBusVInput Bin annotation (
        Placement(transformation(extent = {{18, -4}, {38, 16}})));
      Modelica.Blocks.Interfaces.RealInput v_re(start = 4000 / sqrt(3)) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 0, origin = {-100, 26})));
      Modelica.Blocks.Interfaces.RealInput v_im(start = 3000 / sqrt(3)) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 0, origin = {-100, -10})));
    equation
      connect(Bin.terminal, load.terminal) annotation (
        Line(points = {{31, 6}, {48, 6}, {48, 6.02}, {65.96, 6.02}}, color = {0, 0, 0}));
      connect(v_re, Bin.v_re) annotation (
        Line(points = {{-100, 26}, {-18, 26}, {-18, 2}, {26.2, 2}}, color = {0, 0, 127}));
      connect(Bin.v_im, v_im) annotation (
        Line(points = {{26.2, -2}, {-6, -2}, {-6, -10}, {-100, -10}}, color = {0, 0, 127}));
      annotation (
        Diagram(graphics={  Text(extent = {{-106, -22}, {106, -64}}, lineColor = {28, 108, 200}, fontSize = 12, textString = "voltage at the load is 5 kV
and current is 0.577 A
apparent power flowing the load is 5 kVA"), Text(extent = {{10, 54}, {140, 12}}, lineColor = {28, 108, 200}, fontSize = 12, textString = "P = 4 kW, Q = 3 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-220, 58}, {48, 52}}, fontSize = 12, textString = "v_re = 4000/sqrt(3), v_im = 3000/sqrt(3)")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneLoad;

    model OnePerfectLine "Testing a perfect line"
      extends Icons.myExample;
      Components.myLine line(UNom = 5000, Imax = 60, R = 1e-9, X = 0) annotation (
        Placement(visible = true, transformation(origin = {-4, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Buses.myCausalBusVInput Bin annotation (
        Placement(transformation(extent = {{-46, 4}, {-26, 24}})));
      Buses.myCausalBusVOutput Bout annotation (
        Placement(transformation(extent = {{16, 4}, {36, 24}})));
      Modelica.Blocks.Interfaces.RealInput i_re(start = -30) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 180, origin = {104, 34})));
      Modelica.Blocks.Interfaces.RealInput i_im(start = -40) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 180, origin = {104, -2})));
      Modelica.Blocks.Interfaces.RealInput v_re(start = 4000 / sqrt(3)) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 0, origin = {-110, 40})));
      Modelica.Blocks.Interfaces.RealInput v_im(start = 3000 / sqrt(3)) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 0, origin = {-110, 4})));
    equation
      connect(Bin.terminal, line.terminalA) annotation (
        Line(points = {{-33, 14}, {-14, 14}}, color = {0, 0, 0}));
      connect(Bout.terminal, line.terminalB) annotation (
        Line(points = {{23, 14}, {6, 14}}, color = {0, 0, 0}));
      connect(Bout.i_re, i_re) annotation (
        Line(points = {{28, 22}, {58, 22}, {58, 34}, {104, 34}}, color = {0, 0, 127}));
      connect(Bout.i_im, i_im) annotation (
        Line(points = {{28, 18}, {56, 18}, {56, -2}, {104, -2}}, color = {0, 0, 127}));
      connect(v_re, Bin.v_re) annotation (
        Line(points = {{-110, 40}, {-56, 40}, {-56, 10}, {-37.8, 10}}, color = {0, 0, 127}));
      connect(v_im, Bin.v_im) annotation (
        Line(points = {{-110, 4}, {-74, 4}, {-74, 6}, {-37.8, 6}}, color = {0, 0, 127}));
      annotation (
        Diagram(graphics={  Text(extent = {{-58, -12}, {70, -62}}, lineColor = {28, 108, 200}, fontSize = 12, textString = "current in the line is 50 A and voltage is 5 kV 
no voltage drop as the line is perfect
apparent power flowing the line is 433 kVA"), Text(origin = {42.8, 22.3078}, lineColor = {28, 108, 200}, extent = {{-76.8, 19.6922}, {-12.8, 35.6922}}, fontSize = 12, textString = "perfect line:
R very low, and X=G=B=0"), Text(lineColor = {28, 108, 200}, extent = {{30, 60}, {172, 50}}, fontSize = 12, textString = "i_re = -30, i_im = -40"), Text(lineColor = {28, 108, 200}, extent = {{-208, 74}, {20, 66}}, fontSize = 12, textString = "v_re = 4000/sqrt(3), v_im = 3000/sqrt(3)")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OnePerfectLine;

    model OneLine "Testing a normal line"
      extends Icons.myExample;
      Components.myLine line(UNom = 5000, Imax = 60, R = 10, X = 0) annotation (
        Placement(visible = true, transformation(origin = {-4, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Buses.myCausalBusVInput Bin annotation (
        Placement(transformation(extent = {{-46, 2}, {-26, 22}})));
      Buses.myCausalBusVOutput Bout annotation (
        Placement(transformation(extent = {{18, 2}, {38, 22}})));
      Modelica.Blocks.Interfaces.RealInput i_re(start = -30) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 180, origin = {100, 40})));
      Modelica.Blocks.Interfaces.RealInput i_im(start = -40) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 180, origin = {100, 4})));
      Modelica.Blocks.Interfaces.RealInput v_re(start = 5000 / sqrt(3)) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 0, origin = {-112, 34})));
      Modelica.Blocks.Interfaces.RealInput v_im(start = 0) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 0, origin = {-112, -2})));
    equation
      connect(Bin.terminal, line.terminalA) annotation (
        Line(points = {{-33, 12}, {-14, 12}}, color = {0, 0, 0}));
      connect(Bout.terminal, line.terminalB) annotation (
        Line(points = {{25, 12}, {6, 12}}, color = {0, 0, 0}));
      connect(Bout.i_re, i_re) annotation (
        Line(points = {{30, 20}, {52, 20}, {52, 40}, {100, 40}}, color = {0, 0, 127}));
      connect(Bout.i_im, i_im) annotation (
        Line(points = {{30, 16}, {54, 16}, {54, 4}, {100, 4}}, color = {0, 0, 127}));
      connect(v_im, Bin.v_im) annotation (
        Line(points = {{-112, -2}, {-74, -2}, {-74, 4}, {-37.8, 4}}, color = {0, 0, 127}));
      connect(v_re, Bin.v_re) annotation (
        Line(points = {{-112, 34}, {-76, 34}, {-76, 8}, {-37.8, 8}}, color = {0, 0, 127}));
      annotation (
        Diagram(graphics={  Text(origin = {38.8, 18.3078}, lineColor = {28, 108, 200}, extent = {{-76.8, 19.6922}, {-12.8, 35.6922}}, fontSize = 12, textString = "resistive line:
R = 10 ohms, and X=G=B=0"), Text(extent = {{-58, -18}, {70, -68}}, lineColor = {28, 108, 200}, fontSize = 12, textString = "current in the line is 50 A and voltage is 5 kV 
voltage drop is 466 V
apparent power flowing the line is 433 kVA"), Text(lineColor = {28, 108, 200}, extent = {{26, 66}, {168, 56}}, fontSize = 12, textString = "i_re = -30, i_im = -40"), Text(lineColor = {28, 108, 200}, extent = {{-188, 64}, {-46, 54}}, fontSize = 12, textString = "v_re = 5000/sqrt(3), v_im = 0")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneLine;

    model OneperfectTransfo "Testing a perfect transformer"
      extends Icons.myExample;
      Components.myTransformer tra(UNomA = 63000, UNomB = 20000, SNom = 36000, R = 1e-9) annotation (
        Placement(visible = true, transformation(origin = {-4, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Buses.myCausalBusVInput Bin annotation (
        Placement(transformation(extent = {{-46, 6}, {-26, 26}})));
      Buses.myCausalBusVOutput Bout annotation (
        Placement(transformation(extent = {{16, 6}, {36, 26}})));
      Modelica.Blocks.Interfaces.RealInput i_re(start = -30) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 180, origin = {100, 42})));
      Modelica.Blocks.Interfaces.RealInput i_im(start = -40) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 180, origin = {100, 6})));
      Modelica.Blocks.Interfaces.RealInput v_re(start = 63000 / sqrt(3)) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 0, origin = {-106, 40})));
      Modelica.Blocks.Interfaces.RealInput v_im(start = 0) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 0, origin = {-106, 4})));
    equation
      connect(Bin.terminal, tra.terminalA) annotation (
        Line(points = {{-33, 16}, {-14, 16}}, color = {0, 0, 0}));
      connect(Bout.terminal, tra.terminalB) annotation (
        Line(points = {{23, 16}, {6, 16}}, color = {0, 0, 0}));
      connect(Bout.i_re, i_re) annotation (
        Line(points = {{28, 24}, {54, 24}, {54, 42}, {100, 42}}, color = {0, 0, 127}));
      connect(Bout.i_im, i_im) annotation (
        Line(points = {{28, 20}, {52, 20}, {52, 6}, {100, 6}}, color = {0, 0, 127}));
      connect(v_re, Bin.v_re) annotation (
        Line(points = {{-106, 40}, {-84, 40}, {-84, 12}, {-37.8, 12}}, color = {0, 0, 127}));
      connect(v_im, Bin.v_im) annotation (
        Line(points = {{-106, 4}, {-70, 4}, {-70, 8}, {-37.8, 8}}, color = {0, 0, 127}));
      annotation (
        Diagram(graphics={  Text(origin = {44.8, 20.3078}, lineColor = {28, 108, 200}, extent = {{-76.8, 19.6922}, {-12.8, 35.6922}}, fontSize = 12, textString = "perfect transformer:
R very low, and X=G=B=0
ratio=20/63"), Text(extent = {{-66, -14}, {62, -64}}, lineColor = {28, 108, 200}, fontSize = 12, textString = "voltage at port A is 63 kV and is 20 kV at port B
current in port A is 15.9 A and is 50 A in port B
apparent power flowing the transformer is 1732 kVA"), Text(lineColor = {28, 108, 200}, extent = {{26, 68}, {168, 58}}, fontSize = 12, textString = "i_re = -30, i_im = -40"), Text(lineColor = {28, 108, 200}, extent = {{-200, 66}, {-6, 62}}, fontSize = 12, textString = "v_re = 63000/sqrt(3), v_im = 0")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneperfectTransfo;

    model OneTransfo "Testing a normal transformer"
      extends Icons.myExample;
      Components.myTransformer tra(UNomA = 63000, UNomB = 20000, SNom = 36000, R = 10, X = 0) annotation (
        Placement(visible = true, transformation(origin = {-4, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Buses.myCausalBusVInput Bin annotation (
        Placement(transformation(extent = {{-46, 6}, {-26, 26}})));
      Buses.myCausalBusVOutput Bout annotation (
        Placement(transformation(extent = {{16, 6}, {36, 26}})));
      Modelica.Blocks.Interfaces.RealInput i_re(start = -50) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 180, origin = {100, 50})));
      Modelica.Blocks.Interfaces.RealInput i_im(start = 0) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 180, origin = {100, 14})));
      Modelica.Blocks.Interfaces.RealInput v_re(start = 63000 / sqrt(3)) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 0, origin = {-104, 44})));
      Modelica.Blocks.Interfaces.RealInput v_im(start = 0) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 0, origin = {-104, 8})));
    equation
      connect(Bin.terminal, tra.terminalA) annotation (
        Line(points = {{-33, 16}, {-14, 16}}, color = {0, 0, 0}));
      connect(Bout.terminal, tra.terminalB) annotation (
        Line(points = {{23, 16}, {6, 16}}, color = {0, 0, 0}));
      connect(Bout.i_re, i_re) annotation (
        Line(points = {{28, 24}, {54, 24}, {54, 50}, {100, 50}}, color = {0, 0, 127}));
      connect(Bout.i_im, i_im) annotation (
        Line(points = {{28, 20}, {52, 20}, {52, 14}, {100, 14}}, color = {0, 0, 127}));
      connect(v_re, Bin.v_re) annotation (
        Line(points = {{-104, 44}, {-70, 44}, {-70, 12}, {-37.8, 12}}, color = {0, 0, 127}));
      connect(v_im, Bin.v_im) annotation (
        Line(points = {{-104, 8}, {-37.8, 8}}, color = {0, 0, 127}));
      annotation (
        Diagram(graphics={  Text(origin = {40.8, 24.3078}, lineColor = {28, 108, 200}, extent = {{-76.8, 19.6922}, {-12.8, 35.6922}}, fontSize = 12, textString = "resistive transformer:
R=10 ohms, and X=G=B=0
ratio=20/63"), Text(extent = {{-132, -8}, {142, -58}}, lineColor = {28, 108, 200}, fontSize = 12, textString = "voltage at port A is 63 kV
voltage drop at port B is 87 V bellow 20 kV
current in port A is 15.9 A and is 50 A in port B
apparent power flowing the transformer is 1732 kVA"), Text(lineColor = {28, 108, 200}, extent = {{24, 76}, {166, 66}}, fontSize = 12, textString = "i_re = -50, i_im = 0"), Text(lineColor = {28, 108, 200}, extent = {{-198, 70}, {-4, 66}}, fontSize = 12, textString = "v_re = 63000/sqrt(3), v_im = 0")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneTransfo;

    model OneBank "Testing a capacitor bank"
      extends Icons.myExample;
      Components.myCapacitorBank bank(UNom = 5000, B = 1) annotation (
        Placement(transformation(extent = {{50, 2}, {70, 22}})));
      Buses.myCausalBusVInput Bin annotation (
        Placement(transformation(extent = {{20, 2}, {40, 22}})));
      Modelica.Blocks.Interfaces.RealInput v_re(start = 4000 / sqrt(3)) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 0, origin = {-90, 44})));
      Modelica.Blocks.Interfaces.RealInput v_im(start = 3000 / sqrt(3)) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 0, origin = {-90, 8})));
    equation
      connect(Bin.terminal, bank.terminal) annotation (
        Line(points = {{33, 12}, {60, 12}}, color = {0, 0, 0}));
      connect(v_re, Bin.v_re) annotation (
        Line(points = {{-90, 44}, {-30, 44}, {-30, 8}, {28.2, 8}}, color = {0, 0, 127}));
      connect(v_im, Bin.v_im) annotation (
        Line(points = {{-90, 8}, {-32, 8}, {-32, 4}, {28.2, 4}}, color = {0, 0, 127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false), graphics={  Text(lineColor = {28, 108, 200}, extent = {{28, 44}, {104, 22}}, fontSize = 12, textString = "B = 1 S"), Text(extent = {{-38, -22}, {36, -66}}, lineColor = {28, 108, 200}, fontSize = 12, textString = "voltage at the bank is 5 kV
current is 2887 A
apparent power flowing the bank is 25 MVAR"), Text(lineColor = {28, 108, 200}, extent = {{-242, 74}, {62, 66}}, fontSize = 12, textString = "v_re = 4000/sqrt(3), v_im = 3000/sqrt(3)")}),
        experiment(StopTime = 1));
    end OneBank;

    model VoltageRegulation1 "Testing the voltage regulation"
      extends Icons.myExample;
      parameter Real table[:, 2] = [0, 20400; 10, 20400; 10, 21400; 20, 21400; 20, 20400; 30, 20400; 30, 21400; 91, 21400; 91, 20400; 120, 20400; 120, 18400; 181, 18400; 181, 19400; 191, 19400; 191, 20400; 250, 20400; 250, 21400; 311, 21400; 311, 21350; 321, 21350; 321, 21300; 331, 21300; 331, 21250; 341, 21250; 341, 20400] "Voltage variation";
      Modelica.Blocks.Sources.TimeTable voltage(table = table) annotation (
        Placement(transformation(extent = {{-52, 66}, {-32, 86}})));
      Regulations.myTapChanger tapChanger annotation (
        Placement(visible = true, transformation(origin = {42, 68}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      connect(voltage.y, tapChanger.U) annotation (
        Line(points = {{-31, 76}, {-22, 76}, {-22, 74.4}, {20, 74.4}}, color = {0, 0, 127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false), graphics={  Text(lineColor = {28, 108, 200}, extent = {{-94, 0}, {30, -74}}, fontSize = 12,
                horizontalAlignment =                                                                                                                                            TextAlignment.Left, textString = "voltage evolution:
t=0 s 
t=10 s
t=20 s 
t=30 s
t=91 s
t=120 s 
t=181 s
t=191 s 
t=250 s
t=311 s
t=321 s
t=331 s
t=341 s"), Text(lineColor = {28, 108, 200}, extent = {{-60, 0}, {64, -74}}, fontSize = 12,
                horizontalAlignment =                                                                            TextAlignment.Left, textString = "
U=20400 V
U=21400 V
U=20400 V
U=21400 V 
U=20400 V
U=18400 V
U=19400 V
U=20400 V
U=21400 V
U=21350 V
U=21300 V
U=21250 V
U=20400 V"), Text(lineColor = {28, 108, 200}, extent = {{-2, -2}, {122, -76}}, fontSize = 12,
                horizontalAlignment =                                                                               TextAlignment.Left, textString = "tap decrease at t=90 s,

tap increase at t=180 s, and t=190 s

tap decrease at t=310 s, t=320 s, t=330 s, and t=340 s"), Text(lineColor = {28, 108, 200}, extent = {{-138, 58}, {224, 28}}, fontSize = 12,
                horizontalAlignment =                                                                                                                             TextAlignment.Left, textString = "In this case, the starting voltage is in the normal range [underMinU, aboveMaxU]")}),
        experiment(StopTime = 350));
    end VoltageRegulation1;

    model VoltageRegulation2 "Testing the voltage regulation"
      extends Icons.myExample;
      parameter Real table[:, 2] = [0, 21400; 300, 21400; 300, 20400] "Voltage variation";
      Modelica.Blocks.Sources.TimeTable voltage(table = table) annotation (
        Placement(transformation(extent = {{-52, 66}, {-32, 86}})));
      Regulations.myTapChanger tapChanger annotation (
        Placement(visible = true, transformation(origin = {42, 68}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      connect(voltage.y, tapChanger.U) annotation (
        Line(points = {{-31, 76}, {-22, 76}, {-22, 74.4}, {20, 74.4}}, color = {0, 0, 127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false), graphics={  Text(lineColor = {28, 108, 200}, extent = {{-94, 24}, {30, -50}}, fontSize = 12,
                horizontalAlignment =                                                                                                                                             TextAlignment.Left, textString = "voltage evolution:
t=0 s 
t=300 s"), Text(lineColor = {28, 108, 200}, extent = {{-64, 20}, {60, -54}}, fontSize = 12,
                horizontalAlignment =                                                                             TextAlignment.Left, textString = "U=21400 V
U=20400 V"), Text(lineColor = {28, 108, 200}, extent = {{-2, 20}, {122, -54}}, fontSize = 12,
                horizontalAlignment =                                                                               TextAlignment.Left, textString = "tap decrease at t=60 s, 70 s, 80 s,
90 s, 100 s, 110 s, 120 s, and 130 s"), Text(lineColor = {28, 108, 200}, extent = {{-132, 58}, {230, 28}}, fontSize = 12,
                horizontalAlignment =                                                                                                           TextAlignment.Left, textString = "In this case, the starting voltage is outside the normal range [underMinU, aboveMaxU]")}),
        experiment(StopTime = 350));
    end VoltageRegulation2;

    model QfURegulation "Testing the regulation Q=f(U)"
      extends Icons.myExample;
      import Modelica.Constants.pi;
      Regulations.myQfU qfU(Pracc_inj = 1000, UNom = 1000) annotation (
        Placement(transformation(extent = {{8, 20}, {50, 60}})));
      Modelica.Blocks.Sources.RealExpression sine(y = 1000 + 300 * Modelica.Math.sin(2 * pi * 0.01 * time)) annotation (
        Placement(transformation(extent = {{-190, 62}, {-76, 82}})));
    equation
      connect(qfU.U, sine.y) annotation (
        Line(points = {{5.9, 46.4}, {-30, 46.4}, {-30, 72}, {-70.3, 72}}, color = {0, 0, 127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false), graphics={  Text(lineColor = {28, 108, 200}, extent = {{-68, 16}, {56, -58}}, fontSize = 12,
                horizontalAlignment =                                                                                                                                             TextAlignment.Left, textString = "a variable reactive power is calculated
depending on the variable input voltage")}),
        experiment(StopTime = 150));
    end QfURegulation;
    annotation (
      Documentation(info = "<html>
    <p>This package illustrates the individual behaviour of some important components or regulations.</p>
</body></html>"));
  end Tests;

  package Examples "Some basic examples"
    extends Icons.myExamplesPackage;

    model OneSourceOneLoad
      extends Icons.myExample;
      Components.mySource src(UNom = 10000) annotation (
        Placement(visible = true, transformation(origin = {-26, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load(UNom = 10000, P = 4000, Q = 3000) annotation (
        Placement(visible = true, transformation(origin = {36, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(src.terminal, load.terminal) annotation (
        Line(points = {{-26, 18}, {4, 18}, {4, 18.02}, {35.96, 18.02}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-34, -10}, {40, -54}}, fontSize = 12, textString = "voltage is 10 kV
current is 0,289 A
apparent power flowing the components is 5 kVA"), Text(lineColor = {28, 108, 200}, extent = {{-30, 54}, {134, 28}}, fontSize = 12, textString = "P=4 kW and Q=3 kvar")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceOneLoad;

    model OneSourceOneLoadWithBuses
      extends Icons.myExample;
      Components.mySource src(UNom = 10000) annotation (
        Placement(visible = true, transformation(origin = {-52, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load(UNom = 10000, P = 4000, Q = 3000) annotation (
        Placement(visible = true, transformation(origin = {54, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Buses.myCausalBusVOutput bout annotation (
        Placement(transformation(extent = {{-22, 0}, {-2, 20}})));
      Buses.myCausalBusVInput bin annotation (
        Placement(visible = true, transformation(extent = {{2, 0}, {22, 20}}, rotation = 0)));
    equation
      connect(src.terminal, bout.terminal) annotation (
        Line(points = {{-52, 10}, {-15, 10}}, color = {0, 0, 0}));
      connect(bout.v_re, bin.v_re) annotation (
        Line(points = {{-10, 6}, {10.2, 6}}, color = {0, 0, 127}));
      connect(bout.v_im, bin.v_im) annotation (
        Line(points = {{-10, 2}, {10.2, 2}}, color = {0, 0, 127}));
      connect(bout.i_im, bin.i_im) annotation (
        Line(points = {{-10, 14}, {10.2, 14}}, color = {0, 0, 127}));
      connect(bout.i_re, bin.i_re) annotation (
        Line(points = {{-10, 18}, {10.2, 18}}, color = {0, 0, 127}));
      connect(bin.terminal, load.terminal) annotation (
        Line(points = {{15, 10}, {34, 10}, {34, 10.02}, {53.96, 10.02}}));
      annotation (
        Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{26, 42}, {102, 20}}, fontSize = 12, textString = "P=4 kW and Q=3 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-34, -20}, {40, -64}}, fontSize = 12, textString = "same results as previous model
voltage is 10 kV
current is 0,289 A
apparent power flowing the components is 5 kVA")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceOneLoadWithBuses;

    model OneSourceOneLineOneLoad
      extends Icons.myExample;
      Components.mySource src(UNom = 10000) annotation (
        Placement(visible = true, transformation(origin = {-48, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line(UNom = 10000, Imax = 120, R = 20, X = 0) annotation (
        Placement(visible = true, transformation(origin = {0, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load(P = 5000, UNom = 10000) annotation (
        Placement(visible = true, transformation(origin = {50, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(src.terminal, line.terminalA) annotation (
        Line(points = {{-48, 16}, {-10, 16}}));
      connect(line.terminalB, load.terminal) annotation (
        Line(points = {{10, 16}, {30, 16}, {30, 16.02}, {49.96, 16.02}}));
      annotation (
        Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-22, 52}, {150, 26}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-36, -10}, {38, -54}}, fontSize = 12, textString = "source voltage is 10 kV
current in load is 0.289 A
voltage drop is 10 V in the resistive line")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceOneLineOneLoad;

    model TwoSourcesTwoLinesOneLoad
      extends Icons.myExample;
      Components.mySource src1(UNom = 10000) annotation (
        Placement(visible = true, transformation(origin = {-26, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load(UNom = 10000, P = 4000, Q = 3000) annotation (
        Placement(visible = true, transformation(origin = {36, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.mySource src2(UNom = 10000) annotation (
        Placement(visible = true, transformation(origin = {-26, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line1(UNom = 10000, Imax = 50, R = 1) annotation (
        Placement(visible = true, transformation(origin = {6, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line2(UNom = 10000, Imax = 50, R = 1) annotation (
        Placement(visible = true, transformation(origin = {6, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(src1.terminal, line1.terminalA) annotation (
        Line(points = {{-26, 30}, {-4, 30}}, color = {0, 0, 0}));
      connect(load.terminal, line1.terminalB) annotation (
        Line(points = {{35.96, 10.02}, {26, 10.02}, {26, 30}, {16, 30}}, color = {0, 0, 0}));
      connect(src2.terminal, line2.terminalA) annotation (
        Line(points = {{-26, -12}, {-4, -12}}, color = {0, 0, 0}));
      connect(line2.terminalB, line1.terminalB) annotation (
        Line(points = {{16, -12}, {26, -12}, {26, 30}, {16, 30}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-38, -20}, {36, -64}}, fontSize = 12, textString = "current is 0.289 A in the load
and equally divided in the two lines"), Text(lineColor = {28, 108, 200}, extent = {{4, 36}, {174, 12}}, fontSize = 12, textString = "P=4 kW and Q=3 kvar")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end TwoSourcesTwoLinesOneLoad;

    model OneProdLoadOneLineOneLineOneLoad
      extends Icons.myExample;
      Components.myPVNode pv(Pmax = -10000, UNom = 400) annotation (
        Placement(visible = true, transformation(origin = {24, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load(UNom = 400, P = 5000, Q = 100) annotation (
        Placement(visible = true, transformation(origin = {18, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line1(UNom = 400, Imax = 50, R = 1) annotation (
        Placement(visible = true, transformation(origin = {-4, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line2(UNom = 400, Imax = 50, R = 0.5) annotation (
        Placement(visible = true, transformation(origin = {-4, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(pv.terminal, line1.terminalB) annotation (
        Line(points = {{24, 14}, {6, 14}}, color = {0, 0, 0}));
      connect(line2.terminalB, load.terminal) annotation (
        Line(points = {{6, -12}, {18, -12}, {18, -11.98}, {17.96, -11.98}}, color = {0, 0, 0}));
      connect(line1.terminalA, line2.terminalA) annotation (
        Line(points = {{-14, 14}, {-32, 14}, {-32, -12}, {-14, -12}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-6, 54}, {70, 32}}, textString = "P=5 kW and Q=0.1 kvar for the load
Pstart=-5 kW for the PV node", fontSize = 12), Text(lineColor = {28, 108, 200}, extent = {{-32, -22}, {42, -66}}, fontSize = 12, textString = "the load is correctly supplied by the PV node
and the voltage is correct (380 V)")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneProdLoadOneLineOneLineOneLoad;

    model OneSourceOneTransfo
      extends Icons.myExample;
      Components.mySource src(UNom = 10000) annotation (
        Placement(visible = true, transformation(origin = {-16, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myTransformer tra(UNomA = 10000, UNomB = 5000, R = 0.3, SNom = 0, X = 12.4) annotation (
        Placement(visible = true, transformation(origin = {18, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(src.terminal, tra.terminalA) annotation (
        Line(points = {{-16, 20}, {8, 20}}));
      annotation (
        Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-36, -16}, {38, -60}}, fontSize = 12, textString = "upstream voltage is 10 kV
downstream voltage is 5 kV
current and apparent power are null")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceOneTransfo;

    model OneSourceOneTransfoOneLineOneTransfo
      extends Icons.myExample;
      Components.myTransformer tra1(UNomA = 63000, UNomB = 20000, SNom = 36, R = 0.9, X = 12.4) annotation (
        Placement(visible = true, transformation(origin = {-14, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line(UNom = 20000, Imax = 120, R = 125e-4, X = 125e-4) annotation (
        Placement(visible = true, transformation(origin = {24, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myTransformer tra2(UNomA = 20000, UNomB = 400, SNom = 0.25, R = 0.2, X = 21.22) annotation (
        Placement(visible = true, transformation(origin = {58, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.mySource src(UNom = 63000) annotation (
        Placement(visible = true, transformation(origin = {-52, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(line.terminalB, tra2.terminalA) annotation (
        Line(points = {{34, 12}, {48, 12}}));
      connect(src.terminal, tra1.terminalA) annotation (
        Line(points = {{-52, 12}, {-24, 12}}, color = {0, 0, 0}));
      connect(tra1.terminalB, line.terminalA) annotation (
        Line(points = {{-4, 12}, {14, 12}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics={  Text(extent = {{-36, -16}, {38, -60}}, lineColor = {28, 108, 200}, fontSize = 12, textString = "source voltage is 63 kV
downstream the first transformer the voltage is 20 kV
downstream the second transformer the voltage is 400 V
current is null along the feeder")}),
        experiment(StopTime = 1));
    end OneSourceOneTransfoOneLineOneTransfo;

    model OneSourceOneTransfoOneLineOneTransfoThreeLines
      extends Icons.myExample;
      Components.myTransformer tra1(UNomA = 63000, UNomB = 20000, SNom = 36, R = 0.9, X = 12.4) annotation (
        Placement(visible = true, transformation(origin = {-32, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line1(UNom = 20000, Imax = 20, R = 125e-4, X = 125e-4) annotation (
        Placement(visible = true, transformation(origin = {0, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myTransformer tra2(UNomA = 20000, UNomB = 400, SNom = 0.1, R = 0.2, X = 21.22) annotation (
        Placement(visible = true, transformation(origin = {32, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line2(UNom = 400, Imax = 120, R = 2.157870e-02, X = 2.657870e-02) annotation (
        Placement(visible = true, transformation(origin = {76, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line3(UNom = 400, Imax = 120, R = 2.157870e-02, X = 2.657870e-02) annotation (
        Placement(visible = true, transformation(origin = {76, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line4(UNom = 400, Imax = 120, R = 2.157870e-02, X = 2.657870e-02) annotation (
        Placement(visible = true, transformation(origin = {76, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.mySource src(UNom = 63000) annotation (
        Placement(visible = true, transformation(origin = {-62, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(line1.terminalB, tra2.terminalA) annotation (
        Line(points = {{10, 20}, {22, 20}}));
      connect(tra2.terminalB, line3.terminalA) annotation (
        Line(points = {{42, 20}, {66, 20}}, color = {0, 0, 0}));
      connect(src.terminal, tra1.terminalA) annotation (
        Line(points = {{-62, 20}, {-42, 20}}, color = {0, 0, 0}));
      connect(tra1.terminalB, line1.terminalA) annotation (
        Line(points = {{-22, 20}, {-10, 20}}, color = {0, 0, 0}));
      connect(tra2.terminalB, line2.terminalA) annotation (
        Line(points = {{42, 20}, {48, 20}, {48, 46}, {66, 46}}, color = {0, 0, 0}));
      connect(tra2.terminalB, line4.terminalA) annotation (
        Line(points = {{42, 20}, {48, 20}, {48, -4}, {66, -4}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics={  Text(extent = {{-36, -20}, {38, -64}}, lineColor = {28, 108, 200}, fontSize = 12, textString = "source voltage is 63 kV
downstream the first transformer the voltage is 20 kV
downstream the second transformer the voltage is 400 V
current is null along the feeders")}),
        experiment(StopTime = 1));
    end OneSourceOneTransfoOneLineOneTransfoThreeLines;

    model OneSourceOneTransfoOneLoad
      extends Icons.myExample;
      Components.myTransformer tra(UNomA = 20000, UNomB = 10000, SNom = 0.25, R = 10, X = 0) annotation (
        Placement(visible = true, transformation(origin = {-6, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.mySource src(UNom = 20000) annotation (
        Placement(visible = true, transformation(origin = {-46, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load(P = 5000, UNom = 10000) annotation (
        Placement(visible = true, transformation(origin = {32, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(src.terminal, tra.terminalA) annotation (
        Line(points = {{-46, 20}, {-16, 20}}));
      connect(tra.terminalB, load.terminal) annotation (
        Line(points = {{4, 20}, {18, 20}, {18, 20.02}, {31.96, 20.02}}));
      annotation (
        Diagram(graphics={  Text(origin = {-29.513, 13}, lineColor = {28, 108, 200}, extent = {{-84.5405, -27}, {99.4603, -81}}, textString = "upstream voltage is 20 kV
downstream voltage is 10 kV
current in load is 0.289 A
current in source is 0.144 A
voltage drop at the port B of the transformer is 1.25 V", fontSize = 12), Text(lineColor = {28, 108, 200}, extent = {{-14, 52}, {144, 32}}, textString = "P=5 kW and Q=0 kvar", fontSize = 12)}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceOneTransfoOneLoad;

    model OneSourceThreeLoads
      extends Icons.myExample;
      Components.myLoad load1(UNom = 10000, P = 5000) annotation (
        Placement(visible = true, transformation(origin = {22, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line1(UNom = 10000, Imax = 50, R = 10) annotation (
        Placement(visible = true, transformation(origin = {-6, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load2(UNom = 10000, P = 5000) annotation (
        Placement(visible = true, transformation(origin = {22, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line3(UNom = 10000, Imax = 50, R = 10) annotation (
        Placement(visible = true, transformation(origin = {-6, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load3(UNom = 10000, P = 5000) annotation (
        Placement(visible = true, transformation(origin = {22, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.mySource src(UNom = 10000) annotation (
        Placement(visible = true, transformation(origin = {-66, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line2(UNom = 10000, Imax = 50, R = 10) annotation (
        Placement(visible = true, transformation(origin = {-6, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(line1.terminalB, load1.terminal) annotation (
        Line(points = {{4, 48}, {12, 48}, {12, 48.02}, {21.96, 48.02}}));
      connect(line3.terminalB, load3.terminal) annotation (
        Line(points = {{4, 4}, {12, 4}, {12, 4.02}, {21.96, 4.02}}, color = {0, 0, 0}));
      connect(line2.terminalB, load2.terminal) annotation (
        Line(points = {{4, 26}, {12, 26}, {12, 26.02}, {21.96, 26.02}}, color = {0, 0, 0}));
      connect(src.terminal, line2.terminalA) annotation (
        Line(points = {{-66, 26}, {-16, 26}}, color = {0, 0, 0}));
      connect(src.terminal, line1.terminalA) annotation (
        Line(points = {{-66, 26}, {-38, 26}, {-38, 48}, {-16, 48}}, color = {0, 0, 0}));
      connect(src.terminal, line3.terminalA) annotation (
        Line(points = {{-66, 26}, {-38, 26}, {-38, 4}, {-16, 4}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-30, 84}, {150, 60}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-148, -18}, {144, -64}}, fontSize = 12, textString = "source voltage is 10 kV
current in each line is 0.289 A
voltage drop in each line is 5 V
current in the source 0.867 A")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceThreeLoads;

    model OneSourceOneTransfoOnelineOneLoad
      extends Icons.myExample;
      Components.myLine line(UNom = 10000, Imax = 50, R = 10, X = 2.657870e-02) annotation (
        Placement(visible = true, transformation(origin = {18, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.mySource src(UNom = 20000) annotation (
        Placement(visible = true, transformation(origin = {-54, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load(UNom = 10000, P = 5000) annotation (
        Placement(visible = true, transformation(origin = {46, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myTransformer tra(UNomA = 20000, UNomB = 10000, SNom = 100, R = 0.1, X = 12.4) annotation (
        Placement(visible = true, transformation(origin = {-20, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(src.terminal, tra.terminalA) annotation (
        Line(points = {{-54, 22}, {-30, 22}}));
      connect(line.terminalB, load.terminal) annotation (
        Line(points = {{28, 22}, {36, 22}, {36, 22.02}, {45.96, 22.02}}));
      connect(tra.terminalB, line.terminalA) annotation (
        Line(points = {{-10, 22}, {8, 22}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-20, 62}, {152, 28}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-36, -18}, {38, -62}}, fontSize = 12, textString = "source voltage is 20 kV
downstream voltage in transformer is 10 kV
voltage drop is 5 V in the line
current in the load is 0.289 A
current in the source is 0.144 A")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceOneTransfoOnelineOneLoad;

    model OneSourceTwoLinesOneLoad
      extends Icons.myExample;
      Components.mySource src(UNom = 10000) annotation (
        Placement(transformation(extent = {{-62, 0}, {-42, 20}})));
      Components.myLine line1(UNom = 10000, R = 10, X = 2.657870e-02, Imax = 80) annotation (
        Placement(transformation(extent = {{-32, 0}, {-12, 20}})));
      Components.myLine line2(UNom = 10000, R = 10, X = 2.657870e-02, Imax = 80) annotation (
        Placement(transformation(extent = {{10, 0}, {30, 20}})));
      Components.myLoad load(UNom = 10000, P = 4000, Q = 3000) annotation (
        Placement(transformation(extent = {{40, 0}, {60, 20}})));
    equation
      connect(src.terminal, line1.terminalA) annotation (
        Line(points = {{-52, 10}, {-32, 10}}, color = {0, 0, 0}));
      connect(line1.terminalB, line2.terminalA) annotation (
        Line(points = {{-12, 10}, {10, 10}}, color = {0, 0, 0}));
      connect(line2.terminalB, load.terminal) annotation (
        Line(points = {{30, 10}, {40, 10}, {40, 10.02}, {49.96, 10.02}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-30, 62}, {168, 34}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-36, -8}, {38, -52}}, fontSize = 12, textString = "source voltge is 10 kV
voltage drop is about 4 V in each resistive line
current is 0.289 A in the feeder")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceTwoLinesOneLoad;

    model OneSourceOneLineOneTransfoOneLineOneLoad
      extends Icons.myExample;
      Components.myLine line2(UNom = 400, X = 2e-3, Imax = 60, R = 2e-3) annotation (
        Placement(visible = true, transformation(origin = {50, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load(UNom = 400, P = 50000) annotation (
        Placement(visible = true, transformation(origin = {76, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myTransformer tra1(UNomA = 63000, UNomB = 20000, SNom = 100, R = 1e-9, X = 1e-9) annotation (
        Placement(visible = true, transformation(origin = {-46, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line1(UNom = 20000, Imax = 60, R = 0.4, X = 2.5e-02) annotation (
        Placement(visible = true, transformation(origin = {-14, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myTransformer tra2(UNomA = 20000, UNomB = 400, SNom = 100, R = 1e-9, X = 1e-9) annotation (
        Placement(visible = true, transformation(origin = {16, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.mySource src(UNom = 63000) annotation (
        Placement(visible = true, transformation(origin = {-76, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(line2.terminalB, load.terminal) annotation (
        Line(points = {{60, 22}, {72, 22}, {72, 22.02}, {75.96, 22.02}}));
      connect(src.terminal, tra1.terminalA) annotation (
        Line(points = {{-76, 22}, {-56, 22}}, color = {0, 0, 0}));
      connect(tra1.terminalB, line1.terminalA) annotation (
        Line(points = {{-36, 22}, {-24, 22}}, color = {0, 0, 0}));
      connect(line1.terminalB, tra2.terminalA) annotation (
        Line(points = {{-4, 22}, {6, 22}}, color = {0, 0, 0}));
      connect(tra2.terminalB, line2.terminalA) annotation (
        Line(points = {{26, 22}, {40, 22}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{2, 56}, {176, 34}}, fontSize = 12, textString = "P=50 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-40, -10}, {34, -54}}, fontSize = 12, textString = "as the load is strong
current is exceeding the maximum
acceptable value for line2
but voltage is correct")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceOneLineOneTransfoOneLineOneLoad;

    model OneSourceTwoTransfosOneLoad
      extends Icons.myExample;
      Components.myTransformer tra2(UNomA = 20000, UNomB = 400, SNom = 0.25, R = 1e-9, X = 1e-9) annotation (
        Placement(visible = true, transformation(origin = {20, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load(UNom = 400, P = 5000) annotation (
        Placement(visible = true, transformation(origin = {52, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myTransformer tra1(UNomA = 63000, UNomB = 20000, SNom = 36, R = 1e-9, X = 1e-9) annotation (
        Placement(visible = true, transformation(origin = {-24, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.mySource src(UNom = 63000) annotation (
        Placement(visible = true, transformation(origin = {-66, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(src.terminal, tra1.terminalA) annotation (
        Line(points = {{-66, 22}, {-34, 22}}, color = {0, 0, 0}));
      connect(tra2.terminalB, load.terminal) annotation (
        Line(points = {{30, 22}, {56, 22}, {56, 22.02}, {51.96, 22.02}}, color = {0, 0, 0}));
      connect(tra1.terminalB, tra2.terminalA) annotation (
        Line(points = {{-14, 22}, {10, 22}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{50, 58}, {126, 36}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-24, -6}, {50, -50}}, fontSize = 12, textString = "with almost perfect transformers
current is 7.22 A in the load
and 0.046 A in the source")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceTwoTransfosOneLoad;

    model OneSourceOneTransfoOneLineOneTransfoOneLineOneLoad
      extends Icons.myExample;
      Components.myTransformer tra2(UNomA = 20000, UNomB = 400, SNom = 0.25, R = 1e-6, X = 1e-6) annotation (
        Placement(visible = true, transformation(origin = {20, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line2(UNom = 400, Imax = 60, R = 1, X = 2.657870e-02) annotation (
        Placement(visible = true, transformation(origin = {52, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load(UNom = 400, P = 5000) annotation (
        Placement(visible = true, transformation(origin = {78, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line1(UNom = 20000, Imax = 60, R = 1, X = 125e-4) annotation (
        Placement(visible = true, transformation(origin = {-14, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myTransformer tra1(UNomA = 63000, UNomB = 20000, SNom = 70, R = 0.2, X = 1e-6) annotation (
        Placement(visible = true, transformation(origin = {-52, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.mySource src(UNom = 63000) annotation (
        Placement(visible = true, transformation(origin = {-82, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(line1.terminalB, tra2.terminalA) annotation (
        Line(points = {{-4, 22}, {10, 22}}));
      connect(tra2.terminalB, line2.terminalA) annotation (
        Line(points = {{30, 22}, {42, 22}}));
      connect(line2.terminalB, load.terminal) annotation (
        Line(points = {{62, 22}, {72, 22}, {72, 22.02}, {77.96, 22.02}}));
      connect(src.terminal, tra1.terminalA) annotation (
        Line(points = {{-82, 22}, {-62, 22}}, color = {0, 0, 0}));
      connect(tra1.terminalB, line1.terminalA) annotation (
        Line(points = {{-42, 22}, {-24, 22}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-4, 62}, {168, 40}}, textString = "P=5 kW and Q=0 kvar", fontSize = 12), Text(origin = {-15.7297, 7.27273}, lineColor = {28, 108, 200}, extent = {{-94.2703, -5.27273}, {123.73, -63.2727}}, textString = "current is 7.46 A in the load
and 0.047 A in the source", fontSize = 12)}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceOneTransfoOneLineOneTransfoOneLineOneLoad;

    model OneSourceOneTransfoOneLineOneTransfoOneLineOneLoadWithBuses1
      extends Icons.myExample;
      Buses.myCausalBusVOutput bout1 annotation (
        Placement(visible = true, transformation(origin = {-78, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Buses.myCausalBusVInput bin1 annotation (
        Placement(visible = true, transformation(origin = {-54, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myTransformer tra1(UNomA = 63000, UNomB = 20000, SNom = 70, R = 0.2, X = 1e-6) annotation (
        Placement(visible = true, transformation(origin = {-106, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.mySource src(UNom = 63000) annotation (
        Placement(visible = true, transformation(origin = {-132, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myTransformer tra2(UNomA = 20000, UNomB = 400, SNom = 0.25, R = 1e-6, X = 1e-6) annotation (
        Placement(visible = true, transformation(origin = {4, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line2(UNom = 400, Imax = 60, R = 1, X = 2.657870e-02) annotation (
        Placement(visible = true, transformation(origin = {32, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line1(UNom = 20000, Imax = 60, R = 1, X = 125e-4) annotation (
        Placement(visible = true, transformation(origin = {-30, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load(UNom = 400, P = 5000) annotation (
        Placement(visible = true, transformation(origin = {100, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(bout1.i_re, bin1.i_re) annotation (
        Line(points = {{-76, 28}, {-55.8, 28}}, color = {0, 0, 127}));
      connect(bout1.i_im, bin1.i_im) annotation (
        Line(points = {{-76, 24}, {-55.8, 24}}, color = {0, 0, 127}));
      connect(bin1.v_re, bout1.v_re) annotation (
        Line(points = {{-55.8, 16}, {-76, 16}}, color = {0, 0, 127}));
      connect(bout1.v_im, bin1.v_im) annotation (
        Line(points = {{-76, 12}, {-55.8, 12}}, color = {0, 0, 127}));
      connect(src.terminal, tra1.terminalA) annotation (
        Line(points = {{-132, 20}, {-116, 20}}, color = {0, 0, 0}));
      connect(tra1.terminalB, bout1.terminal) annotation (
        Line(points = {{-96, 20}, {-81, 20}}, color = {0, 0, 0}));
      connect(line1.terminalB, tra2.terminalA) annotation (
        Line(points = {{-20, 20}, {-6, 20}}));
      connect(tra2.terminalB, line2.terminalA) annotation (
        Line(points = {{14, 20}, {22, 20}}));
      connect(bin1.terminal, line1.terminalA) annotation (
        Line(points = {{-51, 20}, {-40, 20}}, color = {0, 0, 0}));
      connect(line2.terminalB, load.terminal) annotation (
        Line(points = {{42, 20}, {70, 20}, {70, 20.02}, {99.96, 20.02}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-28, 58}, {174, 34}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-38, -16}, {36, -60}}, fontSize = 12, textString = "same results as previous model
current is 7.46 A in the load
and 0.047 A in the source")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceOneTransfoOneLineOneTransfoOneLineOneLoadWithBuses1;

    model OneSourceOneTransfoOneLineOneTransfoOneLineOneLoadWithBuses2
      extends Icons.myExample;
      Buses.myCausalBusVOutput bout2 annotation (
        Placement(visible = true, transformation(origin = {56, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Buses.myCausalBusVInput bin2 annotation (
        Placement(visible = true, transformation(origin = {82, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myTransformer tra2(UNomA = 20000, UNomB = 400, SNom = 0.25, R = 1e-6, X = 1e-6) annotation (
        Placement(visible = true, transformation(origin = {4, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line2(UNom = 400, Imax = 60, R = 1, X = 2.657870e-02) annotation (
        Placement(visible = true, transformation(origin = {32, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load(UNom = 400, P = 5000) annotation (
        Placement(visible = true, transformation(origin = {100, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line1(UNom = 20000, Imax = 60, R = 1, X = 125e-4) annotation (
        Placement(visible = true, transformation(origin = {-30, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myTransformer tra1(UNomA = 63000, UNomB = 20000, SNom = 70, R = 0.2, X = 1e-6) annotation (
        Placement(visible = true, transformation(origin = {-106, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.mySource src(UNom = 63000) annotation (
        Placement(visible = true, transformation(origin = {-132, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(bout2.i_re, bin2.i_re) annotation (
        Line(points = {{58, 28}, {80.2, 28}}, color = {0, 0, 127}));
      connect(bout2.i_im, bin2.i_im) annotation (
        Line(points = {{58, 24}, {80.2, 24}}, color = {0, 0, 127}));
      connect(bin2.v_re, bout2.v_re) annotation (
        Line(points = {{80.2, 16}, {58, 16}}, color = {0, 0, 127}));
      connect(bout2.v_im, bin2.v_im) annotation (
        Line(points = {{58, 12}, {80.2, 12}}, color = {0, 0, 127}));
      connect(line1.terminalB, tra2.terminalA) annotation (
        Line(points = {{-20, 20}, {-6, 20}}));
      connect(tra2.terminalB, line2.terminalA) annotation (
        Line(points = {{14, 20}, {22, 20}}));
      connect(src.terminal, tra1.terminalA) annotation (
        Line(points = {{-132, 20}, {-116, 20}}, color = {0, 0, 0}));
      connect(line2.terminalB, bout2.terminal) annotation (
        Line(points = {{42, 20}, {53, 20}}, color = {0, 0, 0}));
      connect(bin2.terminal, load.terminal) annotation (
        Line(points = {{85, 20}, {92, 20}, {92, 20.02}, {99.96, 20.02}}, color = {0, 0, 0}));
      connect(tra1.terminalB, line1.terminalA) annotation (
        Line(points = {{-96, 20}, {-40, 20}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-28, 58}, {174, 34}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-38, -16}, {36, -60}}, fontSize = 12, textString = "same results as previous model
current is 7.46 A in the load
and 0.047 A in the source")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceOneTransfoOneLineOneTransfoOneLineOneLoadWithBuses2;

    model OneSourceOneTransfoOneLineOneTransfoOneLineOneLoadWithBuses3
      extends Icons.myExample;
      Buses.myCausalBusVOutput bout2 annotation (
        Placement(visible = true, transformation(origin = {56, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Buses.myCausalBusVInput bin2 annotation (
        Placement(visible = true, transformation(origin = {82, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myTransformer tra2(UNomA = 20000, UNomB = 400, SNom = 0.25, R = 1e-6, X = 1e-6) annotation (
        Placement(visible = true, transformation(origin = {4, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line2(UNom = 400, Imax = 60, R = 1, X = 2.657870e-02) annotation (
        Placement(visible = true, transformation(origin = {32, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load(UNom = 400, P = 5000) annotation (
        Placement(visible = true, transformation(origin = {100, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line1(UNom = 20000, Imax = 60, R = 1, X = 125e-4) annotation (
        Placement(visible = true, transformation(origin = {-30, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myTransformer tra1(UNomA = 63000, UNomB = 20000, SNom = 70, R = 0.2, X = 1e-6) annotation (
        Placement(visible = true, transformation(origin = {-106, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.mySource src(UNom = 63000) annotation (
        Placement(visible = true, transformation(origin = {-132, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Buses.myCausalBusVOutput bout1 annotation (
        Placement(visible = true, transformation(origin = {-78, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Buses.myCausalBusVInput bin1 annotation (
        Placement(visible = true, transformation(origin = {-54, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(bout2.i_re, bin2.i_re) annotation (
        Line(points = {{58, 28}, {80.2, 28}}, color = {0, 0, 127}));
      connect(bout2.i_im, bin2.i_im) annotation (
        Line(points = {{58, 24}, {80.2, 24}}, color = {0, 0, 127}));
      connect(bin2.v_re, bout2.v_re) annotation (
        Line(points = {{80.2, 16}, {58, 16}}, color = {0, 0, 127}));
      connect(bout2.v_im, bin2.v_im) annotation (
        Line(points = {{58, 12}, {80.2, 12}}, color = {0, 0, 127}));
      connect(line1.terminalB, tra2.terminalA) annotation (
        Line(points = {{-20, 20}, {-6, 20}}));
      connect(tra2.terminalB, line2.terminalA) annotation (
        Line(points = {{14, 20}, {22, 20}}));
      connect(src.terminal, tra1.terminalA) annotation (
        Line(points = {{-132, 20}, {-116, 20}}, color = {0, 0, 0}));
      connect(line2.terminalB, bout2.terminal) annotation (
        Line(points = {{42, 20}, {53, 20}}, color = {0, 0, 0}));
      connect(bin2.terminal, load.terminal) annotation (
        Line(points = {{85, 20}, {92, 20}, {92, 20.02}, {99.96, 20.02}}, color = {0, 0, 0}));
      connect(bout1.i_re, bin1.i_re) annotation (
        Line(points = {{-76, 28}, {-55.8, 28}}, color = {0, 0, 127}));
      connect(bout1.i_im, bin1.i_im) annotation (
        Line(points = {{-76, 24}, {-55.8, 24}}, color = {0, 0, 127}));
      connect(bin1.v_re, bout1.v_re) annotation (
        Line(points = {{-55.8, 16}, {-76, 16}}, color = {0, 0, 127}));
      connect(bout1.v_im, bin1.v_im) annotation (
        Line(points = {{-76, 12}, {-55.8, 12}}, color = {0, 0, 127}));
      connect(tra1.terminalB, bout1.terminal) annotation (
        Line(points = {{-96, 20}, {-81, 20}}, color = {0, 0, 0}));
      connect(bin1.terminal, line1.terminalA) annotation (
        Line(points = {{-51, 20}, {-40, 20}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-28, 58}, {174, 34}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-38, -16}, {36, -60}}, fontSize = 12, textString = "same results as previous model
current is 7.46 A in the load
and 0.047 A in the source")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceOneTransfoOneLineOneTransfoOneLineOneLoadWithBuses3;

    model OneSourceOneLineOneLoadOneBank
      extends Icons.myExample;
      Components.myLoad load(UNom = 20000, P = 4000000, Q = 3000000) annotation (
        Placement(transformation(extent = {{50, -28}, {70, -8}})));
      Components.mySource src(UNom = 20000) annotation (
        Placement(transformation(extent = {{-36, -12}, {-16, 8}})));
      Components.myLine line(UNom = 20000, Imax = 150, R = 2, X = 2) annotation (
        Placement(transformation(extent = {{-2, -12}, {18, 8}})));
      Components.myCapacitorBank bank(UNom = 20000, B = 0.02) annotation (
        Placement(transformation(extent = {{50, 6}, {70, 26}})));
    equation
      connect(src.terminal, line.terminalA) annotation (
        Line(points = {{-26, -2}, {-2, -2}}, color = {0, 0, 0}));
      connect(line.terminalB, bank.terminal) annotation (
        Line(points = {{18, -2}, {40, -2}, {40, 16}, {60, 16}}, color = {0, 0, 0}));
      connect(line.terminalB, load.terminal) annotation (
        Line(points = {{18, -2}, {40, -2}, {40, -17.98}, {59.96, -17.98}}, color = {0, 0, 0}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false), graphics={  Text(lineColor = {28, 108, 200}, extent = {{28, 46}, {104, 24}}, fontSize = 12, textString = "about 8 Mvar"), Text(lineColor = {28, 108, 200}, extent = {{-28, -24}, {174, -46}}, fontSize = 12, textString = "P=4 MW and Q=3 Mvar"), Text(lineColor = {28, 108, 200}, extent = {{-148, -50}, {148, -84}}, fontSize = 12, textString = "voltage after the line is increased
by the capacitor bank
but the current in the line exceeds the maximum admissible value")}),
        experiment(StopTime = 1));
    end OneSourceOneLineOneLoadOneBank;

    model OneSouceOneLineOneSource
      extends Icons.myExample;
      Components.mySource src2(UNom = 9000) annotation (
        Placement(visible = true, transformation(origin = {52, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.mySource src1(UNom = 10000) annotation (
        Placement(visible = true, transformation(origin = {-52, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line(UNom = 10000, Imax = 50, R = 10) annotation (
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(line.terminalB, src2.terminal) annotation (
        Line(points = {{10, 0}, {52, 0}}));
      connect(src1.terminal, line.terminalA) annotation (
        Line(points = {{-52, 0}, {-10, 0}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-144, -24}, {152, -58}}, fontSize = 12, textString = "current  is 57.7 A in line
and Imax is only 50 A")}),
        experiment(StopTime = 1));
    end OneSouceOneLineOneSource;

    model OneSourceThreeLinesOneSource
      extends Icons.myExample;
      Components.mySource src1(UNom = 1000) annotation (
        Placement(visible = true, transformation(origin = {-98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line2(UNom = 1000, Imax = 60, R = 10, X = 1e-6) annotation (
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line3(UNom = 1000, Imax = 50, R = 10, X = 1e-6) annotation (
        Placement(visible = true, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.mySource src2(UNom = 2000) annotation (
        Placement(visible = true, transformation(origin = {84, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line1(UNom = 1000, Imax = 60, R = 10, X = 1e-6) annotation (
        Placement(visible = true, transformation(origin = {0, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(src1.terminal, line2.terminalA) annotation (
        Line(points = {{-98, 0}, {-10, 0}}, color = {0, 0, 0}));
      connect(line3.terminalA, src1.terminal) annotation (
        Line(points = {{-10, -40}, {-62, -40}, {-62, 0}, {-98, 0}}, color = {0, 0, 0}));
      connect(line2.terminalB, src2.terminal) annotation (
        Line(points = {{10, 0}, {50, 0}, {50, 1.33227e-15}, {84, 1.33227e-15}, {84, 0}}, color = {0, 0, 0}));
      connect(line3.terminalB, src2.terminal) annotation (
        Line(points = {{10, -40}, {60, -40}, {60, 0}, {84, 0}}, color = {0, 0, 0}));
      connect(src1.terminal, line1.terminalA) annotation (
        Line(points = {{-98, 0}, {-62, 0}, {-62, 42}, {-10, 42}}, color = {0, 0, 0}));
      connect(line1.terminalB, src2.terminal) annotation (
        Line(points = {{10, 42}, {60, 42}, {60, 1.33227e-15}, {84, 1.33227e-15}, {84, 0}}, color = {0, 0, 0}));
      annotation (
        Diagram(coordinateSystem(initialScale = 0.1), graphics={  Text(lineColor = {28, 108, 200}, extent = {{-142, -48}, {154, -82}}, fontSize = 12, textString = "current in all lines is 57.7 A as line parameters are identical
but Imax for line3 is only 50 A")}),
        experiment(StopTime = 1));
    end OneSourceThreeLinesOneSource;

    model TwoSourcesOneLineThreeSensors
      extends Icons.myExample;
      Components.mySource src2(UNom = 1000) annotation (
        Placement(visible = true, transformation(origin = {60, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.mySource src1(UNom = 10000) annotation (
        Placement(visible = true, transformation(origin = {-60, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line(UNom = 10000, R = 100, X = 1e-6) annotation (
        Placement(visible = true, transformation(origin = {0, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Sensors.WattMeter sensor2 annotation (
        Placement(visible = true, transformation(origin = {32, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Sensors.WattMeter sensor1 annotation (
        Placement(visible = true, transformation(origin = {-34, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Sensors.DiffMeter sensor3 annotation (
        Placement(transformation(extent = {{-10, 48}, {10, 68}})));
    equation
      connect(sensor1.terminalB, line.terminalA) annotation (
        Line(points = {{-24, 28}, {-10, 28}}, color = {0, 0, 0}));
      connect(src1.terminal, sensor1.terminalA) annotation (
        Line(points = {{-60, 28}, {-44, 28}}, color = {0, 0, 0}));
      connect(line.terminalB, sensor2.terminalA) annotation (
        Line(points = {{10, 28}, {22, 28}}, color = {0, 0, 0}));
      connect(sensor2.terminalB, src2.terminal) annotation (
        Line(points = {{42, 28}, {60, 28}}, color = {0, 0, 0}));
      connect(sensor1.Umes, sensor3.UA) annotation (
        Line(points = {{-38, 37}, {-38, 61}, {-11.6, 61}}, color = {0, 0, 127}));
      connect(sensor1.Imes, sensor3.IA) annotation (
        Line(points = {{-30, 37}, {-30, 54.8}, {-11.6, 54.8}}, color = {0, 0, 127}));
      connect(sensor2.Umes, sensor3.UB) annotation (
        Line(points = {{28, 37}, {28, 61}, {11.6, 61}}, color = {0, 0, 127}));
      connect(sensor2.Imes, sensor3.IB) annotation (
        Line(points = {{36, 37}, {36, 55}, {11.6, 55}}, color = {0, 0, 127}));
      annotation (
        Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-150, -14}, {146, -48}}, fontSize = 10, textString = "the two wattmeters measure
current and voltage at port A and port B of the line
sensor3 calculates the drop
in voltage, current and apparent power in the line")}),
        experiment(StopTime = 1));
    end TwoSourcesOneLineThreeSensors;

    model OneSourceOneLigneWithFault
      extends Icons.myExample;
      Components.myLineWithFault lnFault(UNom = 10000, R = 1, Imax = 100, RFault = 0.3, X = 0, faultLocationPu = 0.7, startTime = 0.4, stopTime = 0.6) annotation (
        Placement(visible = true, transformation(origin = {30, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.mySource src(UNom = 10000, theta = 0.5235987755983) annotation (
        Placement(transformation(extent = {{-32, -2}, {-12, 18}})));
    equation
      connect(src.terminal, lnFault.terminalA) annotation (
        Line(points = {{-22, 8}, {20, 8}}, color = {0, 0, 0}));
      annotation (
        Icon(coordinateSystem(grid = {0.1, 0.1})),
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics={  Text(lineColor = {28, 108, 200}, extent = {{-146, -18}, {146, -64}}, fontSize = 12, textString = "a fault appears at 0.4 s and is eliminated in 200 ms
its location is at 70%% line length from port A
during fault, voltage drop at port B is 7 kV
and fault current is very important")}),
        Documentation(info = "<html><head></head><body><p>This model demonstrates the behaviour of a purely resistive transmission line with a purely resistive intermediate fault.</p>
<p>At the beginning of the transient, a current flows from bus A (phase to phase voltage 10 kV) to bus B (phase-to-phase voltage 9 kV) through a resistance of 1 Ohm.</p>
<p>As a consequence, the current flowing in each conductor is</p>
<p>I = (VA - VB)/R = ((UA-UB)/sqrt3)/R = 577.35 A.</p>
<p>When the fault is activated, the analytical computation of the current flowing into the fault yields a current of 10.5215 kA, which corresponds to the simulation result.</p>
<p>At time = 0.6 the fault is cleared and the current through the line returns to the original value.</p>
</body></html>"),
        experiment(StopTime = 1));
    end OneSourceOneLigneWithFault;

    model VariableLoad
      extends Icons.myExample;
      parameter String loadFile = "modelica://PowerSysPro/Resources/Files/variablePQLoad.csv" "External file for PQ variations";
      Modelica.Blocks.Tables.CombiTable1Ds load(tableOnFile = true, tableName = "load", columns = 2:3, fileName = Modelica.Utilities.Files.loadResource(loadFile)) annotation (
        Placement(transformation(extent = {{-12, 10}, {8, 30}})));
      Modelica.Blocks.Sources.RealExpression t(y = time) annotation (
        Placement(transformation(extent = {{-48, 10}, {-28, 30}})));
      Components.myVariableLoad varLoad(UNom = 400) annotation (
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 0, origin = {76, 74})));
      Components.mySource src(UNom = 20000) annotation (
        Placement(visible = true, transformation(origin = {-48, 74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Gain gainP(k = 7000) annotation (
        Placement(transformation(extent = {{38, 34}, {50, 46}})));
      Modelica.Blocks.Math.Gain gainQ(k = 7000) annotation (
        Placement(transformation(extent = {{38, -6}, {50, 6}})));
      Components.myTransformer tra(UNomA = 20000, UNomB = 400, SNom = 0.8, R = 0.1) annotation (
        Placement(transformation(extent = {{2, 64}, {22, 84}})));
      Components.myLine line2(UNom = 400, Imax = 25, R = 1) annotation (
        Placement(transformation(extent = {{38, 64}, {58, 84}})));
      Components.myLine line1(UNom = 20000, Imax = 10, R = 1) annotation (
        Placement(transformation(extent = {{-32, 64}, {-12, 84}})));
    equation
      connect(load.u, t.y) annotation (
        Line(points = {{-14, 20}, {-27, 20}}, color = {0, 0, 127}));
      connect(load.y[1], gainP.u) annotation (
        Line(points = {{9, 20}, {22, 20}, {22, 40}, {36.8, 40}}, color = {0, 0, 127}));
      connect(load.y[2], gainQ.u) annotation (
        Line(points = {{9, 20}, {22, 20}, {22, 0}, {36.8, 0}}, color = {0, 0, 127}));
      connect(gainP.y, varLoad.PInput) annotation (
        Line(points = {{50.6, 40}, {73.46, 40}, {73.46, 65.55}}, color = {0, 0, 127}));
      connect(gainQ.y, varLoad.QInput) annotation (
        Line(points = {{50.6, 0}, {78.565, 0}, {78.565, 65.565}}, color = {0, 0, 127}));
      connect(tra.terminalB, line2.terminalA) annotation (
        Line(points = {{22, 74}, {38, 74}}, color = {0, 0, 0}));
      connect(varLoad.terminal, line2.terminalB) annotation (
        Line(points = {{75.96, 74.02}, {66, 74.02}, {66, 74}, {58, 74}}, color = {0, 0, 0}));
      connect(tra.terminalA, line1.terminalB) annotation (
        Line(points = {{2, 74}, {-12, 74}}, color = {0, 0, 0}));
      connect(src.terminal, line1.terminalA) annotation (
        Line(points = {{-48, 74}, {-32, 74}}, color = {0, 0, 0}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false), graphics={  Text(lineColor = {28, 108, 200}, extent = {{-66, -12}, {58, -86}}, fontSize = 12, textString = "variable P/Q are given by
the resource file variablePQLoad.csv
located in folder Resources/Files
simulation is done for 1-year duration")}),
        experiment(StopTime = 31532400));
    end VariableLoad;

    model QfURegulation "Testing the regulation Q=f(U)"
      extends Icons.myExample;
      Components.myVariableLoad varLoad(UNom = 20000, switchToImpedanceMode = false) annotation (
        Placement(transformation(extent = {{28, 26}, {48, 46}})));
      Components.myLine line(UNom = 20000, R = 1, X = 0, Imax = 60) annotation (
        Placement(transformation(extent = {{-38, 18}, {-18, 38}})));
      Modelica.Blocks.Sources.RealExpression Q(y = 2500) annotation (
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {78, 18})));
      Components.mySource src(UNom = 63000) annotation (
        Placement(transformation(extent = {{-102, 18}, {-82, 38}})));
      Regulations.myQfU qfU(Pracc_inj = 10000, UNom = 20000) annotation (
        Placement(transformation(extent = {{24, 68}, {68, 108}})));
      Sensors.VoltMeter sensor annotation (
        Placement(transformation(extent = {{-2, 56}, {18, 76}})));
      Modelica.Blocks.Sources.RealExpression sine(y = 30000000 * CM.sin(2 * CM.pi * 0.01 * time)) annotation (
        Placement(transformation(extent = {{-106, -16}, {8, 4}})));
      Components.myTransformer tra(UNomA = 63000, UNomB = 20000, R = 1, SNom = 36000) annotation (
        Placement(transformation(extent = {{-72, 18}, {-52, 38}})));
    equation
      connect(Q.y, varLoad.QInput) annotation (
        Line(points = {{67, 18}, {40.565, 18}, {40.565, 27.565}}, color = {0, 0, 127}));
      connect(line.terminalB, varLoad.terminal) annotation (
        Line(points = {{-18, 28}, {20, 28}, {20, 36.02}, {37.96, 36.02}}, color = {0, 0, 0}));
      connect(sensor.Umes, qfU.U) annotation (
        Line(points = {{8, 75}, {8, 94.4}, {21.8, 94.4}}, color = {0, 0, 127}));
      connect(src.terminal, tra.terminalA) annotation (
        Line(points = {{-92, 28}, {-72, 28}}, color = {0, 0, 0}));
      connect(line.terminalA, tra.terminalB) annotation (
        Line(points = {{-38, 28}, {-52, 28}}, color = {0, 0, 0}));
      connect(line.terminalB, sensor.terminal) annotation (
        Line(points = {{-18, 28}, {-10, 28}, {-10, 66}, {-2, 66}}, color = {0, 0, 0}));
      connect(varLoad.PInput, sine.y) annotation (
        Line(points = {{35.46, 27.55}, {35.46, -6}, {13.7, -6}}, color = {0, 0, 127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false), graphics={  Text(lineColor = {28, 108, 200}, extent = {{-64, -2}, {60, -76}}, fontSize = 12, textString = "reactive power control done on
the MV line (injection and consumption)")}),
        experiment(StopTime = 200));
    end QfURegulation;

    package VariableTransformers
      extends Icons.myExamplesPackage;

      model VariableTransformer1 "Comparing transformer with and without tap changer (variable voltage)"
        extends Icons.myExample;
        parameter Real table[:, 2] = [0, 20400; 10, 20400; 10, 21400; 20, 21400; 20, 20400; 30, 20400; 30, 21400; 91, 21400; 91, 20400; 120, 20400; 120, 18400; 181, 18400; 181, 19400; 191, 19400; 191, 20400; 250, 20400; 250, 21400; 311, 21400; 311, 21350; 321, 21350; 321, 21300; 331, 21300; 331, 21250; 341, 21250; 341, 20400] "Voltage variation";
        Modelica.Blocks.Sources.TimeTable voltage(table = table) annotation (
          Placement(transformation(extent = {{-52, 66}, {-32, 86}})));
        variableSource varSrc annotation (
          Placement(visible = true, transformation(origin = {12, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myTransformer fixTra(UNomA = 20000, UNomB = 10000, SNom = 100, R = 1) annotation (
          Placement(visible = true, transformation(origin = {52, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myVariableTransformer varTra(UNomA = 20000, UNomB = 10000, SNom = 100, R = 1) annotation (
          Placement(visible = true, transformation(origin = {52, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(voltage.y, varSrc.UNom) annotation (
          Line(points = {{-31, 76}, {-8, 76}, {-8, 49.15}, {4.45, 49.15}}, color = {0, 0, 127}));
        connect(varSrc.terminal, fixTra.terminalA) annotation (
          Line(points = {{12, 44}, {26, 44}, {26, 62}, {42, 62}}, color = {0, 0, 0}));
        connect(varSrc.terminal, varTra.terminalA) annotation (
          Line(points = {{12, 44}, {26, 44}, {26, 24}, {42, 24}}, color = {0, 0, 0}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false, initialScale = 0.1), graphics={  Text(lineColor = {28, 108, 200}, extent = {{-94, 20}, {30, -54}}, textString = "voltage evolution:
t=0 s 
t=10 s
t=20 s 
t=30 s
t=91 s
t=120 s 
t=181 s
t=191 s 
t=250 s
t=311 s
t=321 s
t=331 s
t=341 s", fontSize = 12, horizontalAlignment = TextAlignment.Left), Text(lineColor = {28, 108, 200}, extent = {{-60, 20}, {64, -54}}, textString = "
U=20400 V
U=21400 V
U=20400 V
U=21400 V 
U=20400 V
U=18400 V
U=19400 V
U=20400 V
U=21400 V
U=21350 V
U=21300 V
U=21250 V
U=20400 V", fontSize = 12, horizontalAlignment = TextAlignment.Left), Text(lineColor = {28, 108, 200}, extent = {{-2, 4}, {122, -70}}, textString = "tap decrease at t=90 s,
 
tap increase at t=180 s, and t=190 s
 
tap decrease at t=310 s, t=320 s, t=330 s, and t=340 s", fontSize = 12,
                  horizontalAlignment =                                                       TextAlignment.Left)}),
          experiment(StopTime = 350));
      end VariableTransformer1;

      model variableSource "Variable voltage"
        parameter Real theta(unit = "rad", displayUnit = "deg") = 0 "Phase of voltage phasor";
        Modelica.Blocks.Interfaces.RealInput UNom(unit = "V", displayUnit = "V") annotation (
          Placement(transformation(extent = {{-120, -20}, {-80, 20}}), iconTransformation(extent = {{-85.5, 41.5}, {-65.5, 61.5}})));
        Interfaces.myAcausalTerminal terminal annotation (
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        terminal.v = CM.fromPolar(UNom * 2 / sqrt(3), theta) "Voltage of ideal generator, phase-to-ground";
        annotation (
          Icon(coordinateSystem(grid = {0.1, 0.1}, initialScale = 0.1), graphics={  Line(origin = {6.37017, 10.33}, points = {{-51.4286, -11.4286}, {-44.9714, 8.11429}, {-40.8571, 18.9143}, {-37.2, 26.5143}, {-33.9429, 31.2}, {-30.7429, 33.7714}, {-27.5429, 34.1714}, {-24.3429, 32.3429}, {-21.0857, 28.4}, {-17.8857, 22.5143}, {-14.2286, 13.7714}, {-9.61714, 0.685714}, {0.0571429, -29.0286}, {4.17143, -40.1143}, {7.82857, -48.1143}, {11.0286, -53.2}, {14.2857, -56.2286}, {17.4857, -57.1429}, {20.6857, -55.7714}, {23.9429, -52.2857}, {27.1429, -46.8}, {30.8, -38.4}, {35.4286, -25.6}, {40, -11.4286}}, smooth = Smooth.Bezier), Text(origin = {-112.04, 0.70086}, rotation = 180, extent = {{-89.9398, 7.90086}, {179.877, -15.7991}}, lineColor = {0, 0, 0}, textStyle = {TextStyle.Bold, TextStyle.Italic}, textString = "variable")}),
          Documentation(info = "<html><head></head><body><p>PVBus: prescribes the line-to-line voltage magnitude <code>U</code> of the bus.</p>
</body></html>"));
      end variableSource;

      model VariableTransformer2 "Comparing transformer with and without tap changer (ramp as voltage)"
        extends Icons.myExample;
        SlackBusRamp slackBusRamp(UNom = 63) annotation (
          Placement(visible = true, transformation(origin = {-58, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myTransformer fixTra(UNomA = 63000, UNomB = 20000, SNom = 100, R = 1) annotation (
          Placement(visible = true, transformation(origin = {-16, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myVariableTransformer varTra(UNomA = 63000, UNomB = 20000, SNom = 100, R = 1) annotation (
          Placement(visible = true, transformation(origin = {-16, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(fixTra.terminalA, slackBusRamp.terminal) annotation (
          Line(points = {{-26, 40}, {-60, 40}, {-60, 20}, {-58, 20}, {-58, 18}}));
        connect(slackBusRamp.terminal, varTra.terminalA) annotation (
          Line(points = {{-58, 18}, {-60, 18}, {-60, -2}, {-26, -2}}));
        annotation (
          experiment(StopTime = 3000),
          Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-58, -2}, {66, -76}}, fontSize = 12, textString = "comparison between fixed transformer
and variable transformer with tap changer")}));
      end VariableTransformer2;

      model SlackBusRamp "Ramp as voltage"
        parameter Types.myVoltage UNom "Voltage magnitude, phase-to-phase";
        parameter Types.myAngle theta = 0 "Phase of voltage phasor";
        Interfaces.myAcausalTerminal terminal annotation (
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Types.myVoltage U(start = UNom);
      equation
        der(1000 * U) = if time < 1000 then -10 else 10;
        terminal.v = CM.fromPolar(1000 * U / sqrt(3), theta) "Voltage of ideal generator, phase-to-ground";
        annotation (
          Icon(coordinateSystem(grid = {0.1, 0.1}, initialScale = 0.1), graphics={  Text(origin = {-119.206, -1.83247}, rotation = 180, extent = {{-55.9061, 7.76753}, {111.812, -15.5324}}, lineColor = {0, 0, 0}, textString = "ramp", textStyle = {TextStyle.Bold, TextStyle.Italic}), Line(origin = {6.37017, 10.33}, points = {{-51.4286, -11.4286}, {-44.9714, 8.11429}, {-40.8571, 18.9143}, {-37.2, 26.5143}, {-33.9429, 31.2}, {-30.7429, 33.7714}, {-27.5429, 34.1714}, {-24.3429, 32.3429}, {-21.0857, 28.4}, {-17.8857, 22.5143}, {-14.2286, 13.7714}, {-9.61714, 0.685714}, {0.0571429, -29.0286}, {4.17143, -40.1143}, {7.82857, -48.1143}, {11.0286, -53.2}, {14.2857, -56.2286}, {17.4857, -57.1429}, {20.6857, -55.7714}, {23.9429, -52.2857}, {27.1429, -46.8}, {30.8, -38.4}, {35.4286, -25.6}, {40, -11.4286}}, smooth = Smooth.Bezier)}));
      end SlackBusRamp;

      model WithVariableLoad
        extends Icons.myExample;
        Components.mySource src(UNom = 63000) annotation (
          Placement(transformation(extent = {{-100, 22}, {-80, 42}})));
        Components.myLine line1(UNom = 63000, R = 10) annotation (
          Placement(transformation(extent = {{-78, 22}, {-58, 42}})));
        Components.myVariableTransformer varTra(UNomA = 63000, UNomB = 20000, R = 1) annotation (
          Placement(transformation(extent = {{-48, 22}, {-28, 42}})));
        Components.myLine line2(UNom = 20000, R = 5) annotation (
          Placement(transformation(extent = {{-14, 22}, {6, 42}})));
        Components.myVariableLoad varLoad(UNom = 20000) annotation (
          Placement(transformation(extent = {{10, 22}, {30, 42}})));
        Modelica.Blocks.Sources.Ramp ramp(height = 9000000, duration = 1000, offset = -100000) annotation (
          Placement(transformation(extent = {{-14, -14}, {14, 14}}, rotation = 180, origin = {42, -18})));
        Modelica.Blocks.Sources.RealExpression Q annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {70, 12})));
      equation
        connect(src.terminal, line1.terminalA) annotation (
          Line(points = {{-90, 32}, {-78, 32}}, color = {0, 0, 0}));
        connect(line1.terminalB, varTra.terminalA) annotation (
          Line(points = {{-58, 32}, {-48, 32}}, color = {0, 0, 0}));
        connect(varTra.terminalB, line2.terminalA) annotation (
          Line(points = {{-28, 32}, {-14, 32}}, color = {0, 0, 0}));
        connect(line2.terminalB, varLoad.terminal) annotation (
          Line(points = {{6, 32}, {12, 32}, {12, 32.02}, {19.96, 32.02}}, color = {0, 0, 0}));
        connect(ramp.y, varLoad.PInput) annotation (
          Line(points = {{26.6, -18}, {17.46, -18}, {17.46, 23.55}}, color = {0, 0, 127}));
        connect(Q.y, varLoad.QInput) annotation (
          Line(points = {{59, 12}, {22.565, 12}, {22.565, 23.565}}, color = {0, 0, 127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false), graphics={  Text(lineColor = {28, 108, 200}, extent = {{-60, -20}, {64, -94}}, fontSize = 12, textString = "tap changes are induced by
the variable active power of the load")}),
          experiment(StopTime = 1000));
      end WithVariableLoad;
    end VariableTransformers;

    package BreakerTests
      extends Icons.myExamplesPackage;

      model BreakersWithLines "Two lines with opposite breakers"
        extends Icons.myExample;
        Components.mySource src2(UNom = 100) annotation (
          Placement(transformation(extent = {{-74, 8}, {-54, 28}})));
        Modelica.Blocks.Logical.Not n annotation (
          Placement(transformation(extent = {{-6, -6}, {6, 6}}, rotation = 180, origin = {-10, 42})));
        Components.mySource src1(UNom = 1000) annotation (
          Placement(transformation(extent = {{-74, 56}, {-54, 76}})));
        Components.myLine line1(UNom = 1000, R = 10, X = 1e-6) annotation (
          Placement(transformation(extent = {{0, 56}, {20, 76}})));
        Components.myLine line2(UNom = 100, R = 10, X = 1e-6) annotation (
          Placement(transformation(extent = {{2, 8}, {22, 28}})));
        Components.myBreaker brk1 annotation (
          Placement(transformation(extent = {{-36, 56}, {-16, 76}})));
        Components.myBreaker brk2 annotation (
          Placement(transformation(extent = {{-36, 8}, {-16, 28}})));
        Modelica.Blocks.Sources.BooleanStep cmd(startTime = 0.5, startValue = true) annotation (
          Placement(transformation(extent = {{-6, -6}, {6, 6}}, rotation = 180, origin = {48, 42})));
        Components.myGround ground annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 0, origin = {86, 44})));
      equation
        connect(src1.terminal, brk1.terminalA) annotation (
          Line(points = {{-64, 66}, {-36, 66}}, color = {0, 0, 0}));
        connect(src2.terminal, brk2.terminalA) annotation (
          Line(points = {{-64, 18}, {-36, 18}}, color = {0, 0, 0}));
        connect(n.y, brk1.BrkOpen) annotation (
          Line(points = {{-16.6, 42}, {-26, 42}, {-26, 63}}, color = {255, 0, 255}));
        connect(cmd.y, n.u) annotation (
          Line(points = {{41.4, 42}, {-2.8, 42}}, color = {255, 0, 255}));
        connect(brk1.terminalB, line1.terminalA) annotation (
          Line(points = {{-16, 66}, {0, 66}}, color = {0, 0, 0}));
        connect(brk2.terminalB, line2.terminalA) annotation (
          Line(points = {{-16, 18}, {2, 18}}, color = {0, 0, 0}));
        connect(line1.terminalB, ground.terminal) annotation (
          Line(points = {{20, 66}, {80, 66}, {80, 44}, {86, 44}}, color = {0, 0, 0}));
        connect(line2.terminalB, ground.terminal) annotation (
          Line(points = {{22, 18}, {80, 18}, {80, 44}, {86, 44}}, color = {0, 0, 0}));
        connect(cmd.y, brk2.BrkOpen) annotation (
          Line(points = {{41.4, 42}, {34, 42}, {34, -2}, {-26, -2}, {-26, 15}}, color = {255, 0, 255}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false), graphics={  Text(lineColor = {28, 108, 200}, extent = {{-122, -6}, {118, -80}}, fontSize = 12, textString = "the positions of the circuit breakers are opposite
brk1 starting position is close
brk2 starting position is open
breaker positions are changing at 0.5 s")}),
          experiment(StopTime = 1));
      end BreakersWithLines;

      model DistrictWithBreakers "District with two breakers"
        extends Icons.myExample;
        Components.myTransformer tra1(UNomA = 20000, UNomB = 400, SNom = 50, R = 0.1) annotation (
          Placement(transformation(extent = {{-18, 38}, {2, 58}})));
        Components.myTransformer tra2(UNomA = 20000, UNomB = 400, SNom = 50, R = 0.1) annotation (
          Placement(transformation(extent = {{-18, -58}, {2, -38}})));
        Components.myLoad load(UNom = 400, P = 5000) annotation (
          Placement(transformation(extent = {{90, -10}, {110, 10}})));
        Components.myLoad load12(UNom = 400, P = 5000) annotation (
          Placement(transformation(extent = {{90, 70}, {110, 90}})));
        Components.myLoad load11(UNom = 400, P = 5000) annotation (
          Placement(transformation(extent = {{90, 38}, {110, 58}})));
        Components.myLoad load21(UNom = 400, P = 5000) annotation (
          Placement(transformation(extent = {{90, -58}, {110, -38}})));
        Components.myLoad load22(UNom = 400, P = 5000) annotation (
          Placement(transformation(extent = {{90, -90}, {110, -70}})));
        Components.myLine line1(UNom = 20000, R = 1) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-44, 28})));
        Components.myLine line2(UNom = 20000, R = 1) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-44, -28})));
        Components.myLine line(UNom = 400, R = 0.1) annotation (
          Placement(transformation(extent = {{58, -10}, {78, 10}})));
        Components.myLine line11(UNom = 400, R = 0.1) annotation (
          Placement(transformation(extent = {{32, 38}, {52, 58}})));
        Components.myLine line111(UNom = 400, R = 0.1) annotation (
          Placement(transformation(extent = {{70, 70}, {90, 90}})));
        Components.myLine line21(UNom = 400, R = 0.1) annotation (
          Placement(transformation(extent = {{32, -58}, {52, -38}})));
        Components.myLine line211(UNom = 400, R = 0.1) annotation (
          Placement(transformation(extent = {{70, -90}, {90, -70}})));
        Components.myBreaker brk1 annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {24, 30})));
        Components.myBreaker brk2 annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {24, -30})));
        Components.mySource src(UNom = 63000) annotation (
          Placement(transformation(extent = {{-100, -10}, {-80, 10}})));
        Components.myTransformer tra(UNomA = 63000, UNomB = 20000, SNom = 100, R = 1) annotation (
          Placement(transformation(extent = {{-72, -10}, {-52, 10}})));
        Modelica.Blocks.Logical.Not n annotation (
          Placement(transformation(extent = {{-6, -6}, {6, 6}}, rotation = 0, origin = {10, -30})));
        Modelica.Blocks.Sources.BooleanStep cmd(startTime = 0.5, startValue = true) annotation (
          Placement(transformation(extent = {{-6, -6}, {6, 6}}, rotation = 0, origin = {-16, 0})));
      equation
        connect(tra1.terminalB, line11.terminalA) annotation (
          Line(points = {{2, 48}, {32, 48}}, color = {0, 0, 0}));
        connect(brk2.terminalA, brk1.terminalB) annotation (
          Line(points = {{24, -20}, {36, -20}, {36, 20}, {24, 20}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line21.terminalA) annotation (
          Line(points = {{2, -48}, {32, -48}}, color = {0, 0, 0}));
        connect(line2.terminalB, tra2.terminalA) annotation (
          Line(points = {{-44, -38}, {-44, -48}, {-18, -48}}, color = {0, 0, 0}));
        connect(brk1.terminalA, line11.terminalA) annotation (
          Line(points = {{24, 40}, {24, 48}, {32, 48}}, color = {0, 0, 0}));
        connect(brk2.terminalB, line21.terminalA) annotation (
          Line(points = {{24, -40}, {24, -48}, {32, -48}}, color = {0, 0, 0}));
        connect(line.terminalA, brk1.terminalB) annotation (
          Line(points = {{58, 0}, {36, 0}, {36, 20}, {24, 20}}, color = {0, 0, 0}));
        connect(load.terminal, line.terminalB) annotation (
          Line(points = {{99.96, 0.02}, {88, 0.02}, {88, 0}, {78, 0}}, color = {0, 0, 0}));
        connect(load21.terminal, line21.terminalB) annotation (
          Line(points = {{99.96, -47.98}, {76, -47.98}, {76, -48}, {52, -48}}, color = {0, 0, 0}));
        connect(load22.terminal, line211.terminalB) annotation (
          Line(points = {{99.96, -79.98}, {96, -79.98}, {96, -80}, {90, -80}}, color = {0, 0, 0}));
        connect(load12.terminal, line111.terminalB) annotation (
          Line(points = {{99.96, 80.02}, {96, 80.02}, {96, 80}, {90, 80}}, color = {0, 0, 0}));
        connect(src.terminal, tra.terminalA) annotation (
          Line(points = {{-90, 0}, {-72, 0}}, color = {0, 0, 0}));
        connect(tra.terminalB, line1.terminalA) annotation (
          Line(points = {{-52, 0}, {-44, 0}, {-44, 18}}, color = {0, 0, 0}));
        connect(tra.terminalB, line2.terminalA) annotation (
          Line(points = {{-52, 0}, {-44, 0}, {-44, -18}}, color = {0, 0, 0}));
        connect(line1.terminalB, tra1.terminalA) annotation (
          Line(points = {{-44, 38}, {-44, 48}, {-18, 48}}, color = {0, 0, 0}));
        connect(line21.terminalB, line211.terminalA) annotation (
          Line(points = {{52, -48}, {60, -48}, {60, -80}, {70, -80}}, color = {0, 0, 0}));
        connect(line11.terminalB, load11.terminal) annotation (
          Line(points = {{52, 48}, {74, 48}, {74, 48.02}, {99.96, 48.02}}, color = {0, 0, 0}));
        connect(line11.terminalB, line111.terminalA) annotation (
          Line(points = {{52, 48}, {60, 48}, {60, 80}, {70, 80}}, color = {0, 0, 0}));
        connect(n.y, brk2.BrkOpen) annotation (
          Line(points = {{16.6, -30}, {21, -30}}, color = {255, 0, 255}));
        connect(cmd.y, n.u) annotation (
          Line(points = {{-9.4, 0}, {0, 0}, {0, -30}, {2.8, -30}}, color = {255, 0, 255}));
        connect(cmd.y, brk1.BrkOpen) annotation (
          Line(points = {{-9.4, 0}, {0, 0}, {0, 30}, {21, 30}}, color = {255, 0, 255}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false), graphics={  Text(lineColor = {28, 108, 200}, extent = {{-160, -42}, {80, -116}}, fontSize = 12, textString = "the position of the circuit breakers are opposite
breaker positions are changing at 0.5 s
all loads are permanently supplied")}),
          experiment(StopTime = 1));
      end DistrictWithBreakers;

      model Islanding1 "Two loads with opposite breakers"
        extends Icons.myExample;
        Components.mySource src(UNom = 20000) annotation (
          Placement(visible = true, transformation(origin = {-88, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myBreaker brk annotation (
          Placement(transformation(extent = {{-10, 20}, {10, 40}})));
        Components.myPVNode pv(UNom = 400, Pmax = -6000) annotation (
          Placement(visible = true, transformation(origin = {68, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load3(UNom = 400, P = 5000, Q = 100) annotation (
          Placement(visible = true, transformation(origin = {62, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line2(UNom = 400, Imax = 50, R = 0.1) annotation (
          Placement(visible = true, transformation(origin = {44, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line3(UNom = 400, Imax = 50, R = 0.1) annotation (
          Placement(visible = true, transformation(origin = {44, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.BooleanStep cmd(startTime = 0.5, startValue = false) annotation (
          Placement(transformation(extent = {{-6, -6}, {6, 6}}, rotation = 0, origin = {-22, -4})));
        Components.myLine line1(UNom = 20000, Imax = 50, R = 0.1) annotation (
          Placement(visible = true, transformation(origin = {-62, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myTransformer tra(UNomA = 20000, UNomB = 400, R = 0.1) annotation (
          Placement(transformation(extent = {{-40, 20}, {-20, 40}})));
      equation
        connect(pv.terminal, line2.terminalB) annotation (
          Line(points = {{68, 42}, {54, 42}}, color = {0, 0, 0}));
        connect(line3.terminalB, load3.terminal) annotation (
          Line(points = {{54, 16}, {62, 16}, {62, 16.02}, {61.96, 16.02}}, color = {0, 0, 0}));
        connect(brk.terminalB, line2.terminalA) annotation (
          Line(points = {{10, 30}, {22, 30}, {22, 42}, {34, 42}}, color = {0, 0, 0}));
        connect(brk.terminalB, line3.terminalA) annotation (
          Line(points = {{10, 30}, {22, 30}, {22, 16}, {34, 16}}, color = {0, 0, 0}));
        connect(cmd.y, brk.BrkOpen) annotation (
          Line(points = {{-15.4, -4}, {0, -4}, {0, 27}}, color = {255, 0, 255}));
        connect(src.terminal, line1.terminalA) annotation (
          Line(points = {{-88, 30}, {-72, 30}}, color = {0, 0, 0}));
        connect(line1.terminalB, tra.terminalA) annotation (
          Line(points = {{-52, 30}, {-40, 30}}, color = {0, 0, 0}));
        connect(tra.terminalB, brk.terminalA) annotation (
          Line(points = {{-20, 30}, {-10, 30}}, color = {0, 0, 0}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false), graphics={  Text(lineColor = {28, 108, 200}, extent = {{-116, -16}, {124, -90}}, fontSize = 12, textString = "the circuit breaker switches off at 0.5 s
consuming load is permanently supplied
by the source and/or by the PV node"), Text(lineColor = {28, 108, 200}, extent = {{36, 78}, {112, 56}}, fontSize = 12, textString = "P=5 kW and Q=0.1 kvar for the load
Pstart=-2 kW for the PV node")}),
          experiment(StopTime = 1));
      end Islanding1;

      model Islanding2 "Two loads with opposite breakers"
        extends Icons.myExample;
        Components.mySource src(UNom = 20000) annotation (
          Placement(visible = true, transformation(origin = {-86, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myBreaker brk annotation (
          Placement(transformation(extent = {{-8, -2}, {12, 18}})));
        Components.myPVNode pv1(UNom = 400, Pmax = -6000) annotation (
          Placement(visible = true, transformation(origin = {86, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load3(UNom = 400, P = 5000, Q = 100) annotation (
          Placement(visible = true, transformation(origin = {80, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line2(UNom = 400, Imax = 50, R = 0.1) annotation (
          Placement(visible = true, transformation(origin = {50, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line3(UNom = 400, Imax = 50, R = 0.1) annotation (
          Placement(visible = true, transformation(origin = {50, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.BooleanStep cmd(startTime = 0.5, startValue = false) annotation (
          Placement(transformation(extent = {{-6, -6}, {6, 6}}, rotation = 0, origin = {-20, -26})));
        Components.myLine line1(UNom = 20000, Imax = 50, R = 0.1) annotation (
          Placement(visible = true, transformation(origin = {-60, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myTransformer tra(UNomA = 20000, UNomB = 400, R = 0.1) annotation (
          Placement(transformation(extent = {{-38, -2}, {-18, 18}})));
        Components.myPVNode pv2(UNom = 400, Pmax = -6000) annotation (
          Placement(visible = true, transformation(origin = {86, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load5(UNom = 400, P = 5000, Q = 100) annotation (
          Placement(visible = true, transformation(origin = {80, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line4(UNom = 400, Imax = 50, R = 0.1) annotation (
          Placement(visible = true, transformation(origin = {50, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line5(UNom = 400, Imax = 50, R = 0.1) annotation (
          Placement(visible = true, transformation(origin = {50, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(pv1.terminal, line2.terminalB) annotation (
          Line(points = {{86, 42}, {60, 42}}, color = {0, 0, 0}));
        connect(line3.terminalB, load3.terminal) annotation (
          Line(points = {{60, 18}, {72, 18}, {72, 18.02}, {79.96, 18.02}}, color = {0, 0, 0}));
        connect(brk.terminalB, line2.terminalA) annotation (
          Line(points = {{12, 8}, {24, 8}, {24, 42}, {40, 42}}, color = {0, 0, 0}));
        connect(brk.terminalB, line3.terminalA) annotation (
          Line(points = {{12, 8}, {24, 8}, {24, 18}, {40, 18}}, color = {0, 0, 0}));
        connect(cmd.y, brk.BrkOpen) annotation (
          Line(points = {{-13.4, -26}, {2, -26}, {2, 5}}, color = {255, 0, 255}));
        connect(src.terminal, line1.terminalA) annotation (
          Line(points = {{-86, 8}, {-70, 8}}, color = {0, 0, 0}));
        connect(line1.terminalB, tra.terminalA) annotation (
          Line(points = {{-50, 8}, {-38, 8}}, color = {0, 0, 0}));
        connect(tra.terminalB, brk.terminalA) annotation (
          Line(points = {{-18, 8}, {-8, 8}}, color = {0, 0, 0}));
        connect(pv2.terminal, line4.terminalB) annotation (
          Line(points = {{86, -2}, {60, -2}}, color = {0, 0, 0}));
        connect(line5.terminalB, load5.terminal) annotation (
          Line(points = {{60, -26}, {72, -26}, {72, -25.98}, {79.96, -25.98}}, color = {0, 0, 0}));
        connect(brk.terminalB, line4.terminalA) annotation (
          Line(points = {{12, 8}, {24, 8}, {24, -2}, {40, -2}}, color = {0, 0, 0}));
        connect(brk.terminalB, line5.terminalA) annotation (
          Line(points = {{12, 8}, {24, 8}, {24, -26}, {40, -26}}, color = {0, 0, 0}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false), graphics={  Text(lineColor = {28, 108, 200}, extent = {{36, 78}, {112, 56}}, fontSize = 12, textString = "P=5 kW and Q=0.1 kvar for the loads
Pstart=-2 kW for the PV nodes"), Text(lineColor = {28, 108, 200}, extent = {{-118, -30}, {122, -104}}, fontSize = 12, textString = "the circuit breaker switches off at 0.5 s
consuming loads are permanently supplied
by the source and/or by the PV node")}),
          experiment(StopTime = 1));
      end Islanding2;
    end BreakerTests;

    package ExampleWithFMUs "Small MV-LV network"
      extends Icons.myExamplesPackage;

      model MVLVNetwork "Whole network"
        extends Icons.myExample;
        Components.myTransformer tra1(UNomA = 63000, UNomB = 20000, SNom = 100000, R = 0.3, X = 1e-9) annotation (
          Placement(visible = true, transformation(origin = {-72, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line1(UNom = 20000, Imax = 60, R = 0.9, X = 0.3) annotation (
          Placement(visible = true, transformation(origin = {-44, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myTransformer tra2(UNomA = 20000, UNomB = 400, SNom = 250, R = 1e-9, X = 1e-9) annotation (
          Placement(visible = true, transformation(origin = {-18, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load1(P = 5000, UNom = 400) annotation (
          Placement(visible = true, transformation(origin = {34, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line2(UNom = 400, Imax = 60, R = 2.157870e-02, X = 2.657870e-02) annotation (
          Placement(visible = true, transformation(origin = {14, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load2(P = 5000, UNom = 400) annotation (
          Placement(visible = true, transformation(origin = {34, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line4(UNom = 400, Imax = 60, R = 2.157870e-02, X = 2.657870e-02) annotation (
          Placement(visible = true, transformation(origin = {14, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load3(P = 5000, UNom = 400) annotation (
          Placement(visible = true, transformation(origin = {34, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.mySource src(UNom = 63000) annotation (
          Placement(visible = true, transformation(origin = {-96, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line3(UNom = 400, Imax = 60, R = 2.157870e-02, X = 2.657870e-02) annotation (
          Placement(visible = true, transformation(origin = {14, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(line2.terminalB, load1.terminal) annotation (
          Line(points = {{24, 38}, {28, 38}, {28, 38.02}, {33.96, 38.02}}));
        connect(line4.terminalB, load3.terminal) annotation (
          Line(points = {{24, 2}, {28, 2}, {28, 2.02}, {33.96, 2.02}}));
        connect(src.terminal, tra1.terminalA) annotation (
          Line(points = {{-96, 20}, {-82, 20}}, color = {0, 0, 0}));
        connect(tra1.terminalB, line1.terminalA) annotation (
          Line(points = {{-62, 20}, {-54, 20}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line2.terminalA) annotation (
          Line(points = {{-8, 20}, {-4, 20}, {-4, 38}, {4, 38}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line4.terminalA) annotation (
          Line(points = {{-8, 20}, {-4, 20}, {-4, 2}, {4, 2}}, color = {0, 0, 0}));
        connect(load2.terminal, line3.terminalB) annotation (
          Line(points = {{33.96, 20.02}, {28, 20.02}, {28, 20}, {24, 20}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line3.terminalA) annotation (
          Line(points = {{-8, 20}, {4, 20}}, color = {0, 0, 0}));
        connect(line1.terminalB, tra2.terminalA) annotation (
          Line(points = {{-34, 20}, {-28, 20}}, color = {0, 0, 0}));
        annotation (
          Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{8, 42}, {186, 2}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-32, -8}, {42, -52}}, fontSize = 12, textString = "current is 7.22 A in each load
and 0.138 A in the source")}, coordinateSystem(initialScale = 0.1)),
          experiment(StopTime = 1));
      end MVLVNetwork;

      model MVLVNetworkWithBuses "Same network with buses"
        extends Icons.myExample;
        Components.myTransformer tra1(UNomA = 63000, UNomB = 20000, SNom = 100000, R = 0.3, X = 1e-9) annotation (
          Placement(visible = true, transformation(origin = {-64, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line1(UNom = 20000, Imax = 60, R = 0.9, X = 0.3) annotation (
          Placement(visible = true, transformation(origin = {-38, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myTransformer tra2(UNomA = 20000, UNomB = 400, SNom = 250, R = 1e-9, X = 1e-9) annotation (
          Placement(visible = true, transformation(origin = {-12, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load1(P = 5000, UNom = 400) annotation (
          Placement(visible = true, transformation(origin = {76, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line2(UNom = 400, Imax = 60, R = 2.157870e-02, X = 2.657870e-02) annotation (
          Placement(visible = true, transformation(origin = {58, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load2(P = 5000, UNom = 400) annotation (
          Placement(visible = true, transformation(origin = {76, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line4(UNom = 400, Imax = 60, R = 2.157870e-02, X = 2.657870e-02) annotation (
          Placement(visible = true, transformation(origin = {58, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load3(P = 5000, UNom = 400) annotation (
          Placement(visible = true, transformation(origin = {76, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.mySource src(UNom = 63000) annotation (
          Placement(visible = true, transformation(origin = {-84, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line3(UNom = 400, Imax = 60, R = 2.157870e-02, X = 2.657870e-02) annotation (
          Placement(visible = true, transformation(origin = {58, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Buses.myCausalBusVOutput bout annotation (
          Placement(visible = true, transformation(origin = {14, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Buses.myCausalBusVInput bin annotation (
          Placement(visible = true, transformation(origin = {26, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(line1.terminalB, tra2.terminalA) annotation (
          Line(points = {{-28, 20}, {-22, 20}}));
        connect(line2.terminalB, load1.terminal) annotation (
          Line(points = {{68, 38}, {72, 38}, {72, 38.02}, {75.96, 38.02}}));
        connect(line4.terminalB, load3.terminal) annotation (
          Line(points = {{68, 2}, {72, 2}, {72, 2.02}, {75.96, 2.02}}));
        connect(src.terminal, tra1.terminalA) annotation (
          Line(points = {{-84, 20}, {-74, 20}}, color = {0, 0, 0}));
        connect(tra1.terminalB, line1.terminalA) annotation (
          Line(points = {{-54, 20}, {-48, 20}}, color = {0, 0, 0}));
        connect(load2.terminal, line3.terminalB) annotation (
          Line(points = {{75.96, 20.02}, {72, 20.02}, {72, 20}, {68, 20}}, color = {0, 0, 0}));
        connect(bout.i_re, bin.i_re) annotation (
          Line(points = {{16, 28}, {24.2, 28}}, color = {0, 0, 127}));
        connect(bout.i_im, bin.i_im) annotation (
          Line(points = {{16, 24}, {24.2, 24}}, color = {0, 0, 127}));
        connect(bin.v_re, bout.v_re) annotation (
          Line(points = {{24.2, 16}, {16, 16}}, color = {0, 0, 127}));
        connect(bout.v_im, bin.v_im) annotation (
          Line(points = {{16, 12}, {24.2, 12}}, color = {0, 0, 127}));
        connect(tra2.terminalB, bout.terminal) annotation (
          Line(points = {{-2, 20}, {11, 20}}, color = {0, 0, 0}));
        connect(bin.terminal, line3.terminalA) annotation (
          Line(points = {{29, 20}, {48, 20}}, color = {0, 0, 0}));
        connect(bin.terminal, line2.terminalA) annotation (
          Line(points = {{29, 20}, {42, 20}, {42, 38}, {48, 38}}, color = {0, 0, 0}));
        connect(bin.terminal, line4.terminalA) annotation (
          Line(points = {{29, 20}, {42, 20}, {42, 2}, {48, 2}}, color = {0, 0, 0}));
        annotation (
          Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-34, -14}, {40, -58}}, fontSize = 12, textString = "same results as previous model
current is 7.22 A in each load
and 0.138 A in the source"), Text(lineColor = {28, 108, 200}, extent = {{38, 46}, {236, -8}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar")}, coordinateSystem(initialScale = 0.1)),
          experiment(StopTime = 1));
      end MVLVNetworkWithBuses;

      model MVPartOfMVLVNetwork "MV part (ready for export as FMU)"
        extends Icons.myExample;
        Components.myTransformer tra1(UNomA = 63000, UNomB = 20000, SNom = 100000, R = 0.3, X = 1e-9) annotation (
          Placement(visible = true, transformation(origin = {-76, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line1(UNom = 20000, Imax = 60, R = 0.9, X = 0.3) annotation (
          Placement(visible = true, transformation(origin = {-48, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myTransformer tra2(UNomA = 20000, UNomB = 400, SNom = 250, R = 1e-9, X = 1e-9) annotation (
          Placement(visible = true, transformation(origin = {-20, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.mySource src(UNom = 63000) annotation (
          Placement(visible = true, transformation(origin = {-96, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Buses.myCausalBusVOutput bout annotation (
          Placement(visible = true, transformation(origin = {6, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput i_re(unit = "A", displayUnit = "A") annotation (
          Placement(transformation(extent = {{-11, -11}, {11, 11}}, rotation = 180, origin = {111, 53}), iconTransformation(extent = {{-11, -11}, {11, 11}}, rotation = 180, origin = {110, 50})));
        Modelica.Blocks.Interfaces.RealInput i_im(unit = "A", displayUnit = "A") annotation (
          Placement(transformation(extent = {{-11, -11}, {11, 11}}, rotation = 180, origin = {111, 35}), iconTransformation(extent = {{-11, -11}, {11, 11}}, rotation = 180, origin = {110, 20})));
        Modelica.Blocks.Interfaces.RealOutput v_re(unit = "V", displayUnit = "V") annotation (
          Placement(transformation(extent = {{100, -20}, {120, 0}}), iconTransformation(extent = {{100, -20}, {120, 0}})));
        Modelica.Blocks.Interfaces.RealOutput v_im(unit = "V", displayUnit = "V") annotation (
          Placement(transformation(extent = {{100, -58}, {120, -38}}), iconTransformation(extent = {{100, -58}, {120, -38}})));
      equation
        connect(line1.terminalB, tra2.terminalA) annotation (
          Line(points = {{-38, 20}, {-30, 20}}));
        connect(src.terminal, tra1.terminalA) annotation (
          Line(points = {{-96, 20}, {-86, 20}}, color = {0, 0, 0}));
        connect(tra1.terminalB, line1.terminalA) annotation (
          Line(points = {{-66, 20}, {-58, 20}}, color = {0, 0, 0}));
        connect(tra2.terminalB, bout.terminal) annotation (
          Line(points = {{-10, 20}, {3, 20}}, color = {0, 0, 0}));
        connect(bout.v_im, v_im) annotation (
          Line(points = {{8, 12}, {32, 12}, {32, -48}, {110, -48}}, color = {0, 0, 127}));
        connect(bout.v_re, v_re) annotation (
          Line(points = {{8, 16}, {48, 16}, {48, -10}, {110, -10}}, color = {0, 0, 127}));
        connect(bout.i_im, i_im) annotation (
          Line(points = {{8, 24}, {84, 24}, {84, 35}, {111, 35}}, color = {0, 0, 127}));
        connect(bout.i_re, i_re) annotation (
          Line(points = {{8, 28}, {68, 28}, {68, 53}, {111, 53}}, color = {0, 0, 127}));
        annotation (
          Diagram(coordinateSystem(initialScale = 0.1), graphics={  Text(lineColor = {28, 108, 200}, extent = {{-124, -42}, {116, -86}}, fontSize = 12, textString = "first model to export as an FMU is
the MV part with causal inputs/outputs")}),
          experiment(StopTime = 1));
      end MVPartOfMVLVNetwork;

      model LVPartOfMVLVNetwork "LV part (ready for export as FMU)"
        extends Icons.myExample;
        Components.myLoad load1(P = 5000, UNom = 400) annotation (
          Placement(visible = true, transformation(origin = {68, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line2(UNom = 400, Imax = 60, R = 2.157870e-02, X = 2.657870e-02) annotation (
          Placement(visible = true, transformation(origin = {48, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load2(P = 5000, UNom = 400) annotation (
          Placement(visible = true, transformation(origin = {68, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line4(UNom = 400, Imax = 60, R = 2.157870e-02, X = 2.657870e-02) annotation (
          Placement(visible = true, transformation(origin = {48, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load3(P = 5000, UNom = 400) annotation (
          Placement(visible = true, transformation(origin = {68, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line3(UNom = 400, Imax = 60, R = 2.157870e-02, X = 2.657870e-02) annotation (
          Placement(visible = true, transformation(origin = {48, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Buses.myCausalBusVInput bin annotation (
          Placement(visible = true, transformation(origin = {16, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput v_re(unit = "V", displayUnit = "V", start = startValues.v_re) annotation (
          Placement(transformation(extent = {{-11, -11}, {11, 11}}, rotation = 0, origin = {-109, 1}), iconTransformation(extent = {{-123, -31}, {-101, -9}})));
        Modelica.Blocks.Interfaces.RealInput v_im(unit = "V", displayUnit = "V", start = startValues.v_im) annotation (
          Placement(transformation(extent = {{-11, -11}, {11, 11}}, rotation = 0, origin = {-109, -17}), iconTransformation(extent = {{-123, -61}, {-101, -39}})));
        Modelica.Blocks.Interfaces.RealOutput i_re(unit = "A", displayUnit = "A") annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-110, 48}), iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-110, 50})));
        Modelica.Blocks.Interfaces.RealOutput i_im(unit = "A", displayUnit = "A") annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-110, 34}), iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-110, 20})));
        parameter sV startValues annotation (
          Placement(transformation(extent = {{2, 40}, {22, 60}})));
      equation
        connect(line2.terminalB, load1.terminal) annotation (
          Line(points = {{58, 38}, {62, 38}, {62, 38.02}, {67.96, 38.02}}));
        connect(line4.terminalB, load3.terminal) annotation (
          Line(points = {{58, 2}, {62, 2}, {62, 2.02}, {67.96, 2.02}}));
        connect(load2.terminal, line3.terminalB) annotation (
          Line(points = {{67.96, 20.02}, {62, 20.02}, {62, 20}, {58, 20}}, color = {0, 0, 0}));
        connect(bin.terminal, line3.terminalA) annotation (
          Line(points = {{19, 20}, {38, 20}}, color = {0, 0, 0}));
        connect(bin.terminal, line2.terminalA) annotation (
          Line(points = {{19, 20}, {30, 20}, {30, 38}, {38, 38}}, color = {0, 0, 0}));
        connect(bin.terminal, line4.terminalA) annotation (
          Line(points = {{19, 20}, {30, 20}, {30, 2}, {38, 2}}, color = {0, 0, 0}));
        connect(i_re, bin.i_re) annotation (
          Line(points = {{-110, 48}, {-64, 48}, {-64, 28}, {14.2, 28}}, color = {0, 0, 127}));
        connect(i_im, bin.i_im) annotation (
          Line(points = {{-110, 34}, {-74, 34}, {-74, 24}, {14.2, 24}}, color = {0, 0, 127}));
        connect(v_im, bin.v_im) annotation (
          Line(points = {{-109, -17}, {-62, -17}, {-62, 12}, {14.2, 12}}, color = {0, 0, 127}));
        connect(v_re, bin.v_re) annotation (
          Line(points = {{-109, 1}, {-74, 1}, {-74, 16}, {14.2, 16}}, color = {0, 0, 127}));
        annotation (
          Diagram(coordinateSystem(initialScale = 0.1), graphics={  Text(lineColor = {28, 108, 200}, extent = {{14, 38}, {248, 4}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-120, -32}, {120, -76}}, fontSize = 12, textString = "second model to export as an FMU is
the LV part with causal inputs/outputs

start value for v_re is set to 230 V
before exporting as an FMU")}),
          experiment(StopTime = 1));
      end LVPartOfMVLVNetwork;

      record sV "start values for the causal input bus"
        extends Icons.myRecord;
        Real v_re = 230 annotation (
          Dialog);
        Real v_im = 0 annotation (
          Dialog);
      end sV;
      annotation (
        Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-140, 36}, {142, -22}}, fontSize = 12, textString = "here is a small MV LV academic network
the same model is then cut in 2 parts to get FMUs")}));
    end ExampleWithFMUs;

    package MediumNetworks "Medium MV-LV network"
      extends Icons.myExamplesPackage;

      model Network "Simple Distribution network"
        extends Icons.myExample;
        Components.myTransformer tra1(UNomA = 63000, UNomB = 20000, SNom = 36000, R = 1e-9, X = 1e-9) annotation (
          Placement(visible = true, transformation(origin = {-62, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line1(UNom = 20000, Imax = 160, R = 0.9, X = 0.3) annotation (
          Placement(visible = true, transformation(origin = {-30, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myTransformer tra2(UNomA = 20000, UNomB = 400, SNom = 250, R = 1e-9, X = 1e-9) annotation (
          Placement(visible = true, transformation(origin = {0, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load1(P = 36000 / sqrt(1.16), UNom = 400, Q = 0.4 * 36000 / sqrt(1.16)) annotation (
          Placement(visible = true, transformation(origin = {78, 74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line2(UNom = 400, Imax = 140, R = 0.2, X = 0.2) annotation (
          Placement(visible = true, transformation(origin = {50, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load2(P = 36000 / sqrt(1.16), UNom = 400, Q = 0.4 * 36000 / sqrt(1.16)) annotation (
          Placement(visible = true, transformation(origin = {80, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line4(UNom = 400, Imax = 140, R = 0.2, X = 0.2) annotation (
          Placement(visible = true, transformation(origin = {50, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load3(P = 36000 / sqrt(1.16), UNom = 400, Q = 0.4 * 36000 / sqrt(1.16)) annotation (
          Placement(visible = true, transformation(origin = {80, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.mySource src(UNom = 63000) annotation (
          Placement(visible = true, transformation(origin = {-88, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line3(UNom = 400, Imax = 140, R = 0.2, X = 0.2) annotation (
          Placement(visible = true, transformation(origin = {50, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line5(UNom = 400, Imax = 140, R = 0.2, X = 0.2) annotation (
          Placement(visible = true, transformation(origin = {88, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load4(P = 36000 / sqrt(1.16), UNom = 400, Q = 0.4 * 36000 / sqrt(1.16)) annotation (
          Placement(visible = true, transformation(origin = {110, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(line1.terminalB, tra2.terminalA) annotation (
          Line(points = {{-20, 16}, {-10, 16}}));
        connect(line2.terminalB, load1.terminal) annotation (
          Line(points = {{60, 38}, {70, 38}, {70, 74.02}, {77.96, 74.02}}));
        connect(line4.terminalB, load3.terminal) annotation (
          Line(points = {{60, -6}, {70, -6}, {70, -5.98}, {79.96, -5.98}}));
        connect(tra1.terminalB, line1.terminalA) annotation (
          Line(points = {{-52, 16}, {-40, 16}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line2.terminalA) annotation (
          Line(points = {{10, 16}, {24, 16}, {24, 38}, {40, 38}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line4.terminalA) annotation (
          Line(points = {{10, 16}, {24, 16}, {24, -6}, {40, -6}}, color = {0, 0, 0}));
        connect(load2.terminal, line3.terminalB) annotation (
          Line(points = {{79.96, 16.02}, {70, 16.02}, {70, 16}, {60, 16}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line3.terminalA) annotation (
          Line(points = {{10, 16}, {40, 16}}, color = {0, 0, 0}));
        connect(line2.terminalB, line5.terminalA) annotation (
          Line(points = {{60, 38}, {78, 38}}, color = {0, 0, 0}));
        connect(line5.terminalB, load4.terminal) annotation (
          Line(points = {{98, 38}, {102, 38}, {102, 38.02}, {109.96, 38.02}}, color = {0, 0, 0}));
        connect(src.terminal, tra1.terminalA) annotation (
          Line(points = {{-88, 16}, {-72, 16}, {-72, 16}}, color = {0, 0, 0}));
        annotation (
          Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-14, 96}, {62, 74}}, fontSize = 10, textStyle = {TextStyle.Bold}, textString = "All residential loads
36 kVA (Q=0.4*P)"), Text(lineColor = {28, 108, 200}, extent = {{-110, 42}, {-66, 28}}, textStyle = {TextStyle.Bold}, fontSize = 10, textString = "RTE"), Text(lineColor = {28, 108, 200}, extent = {{-42, -22}, {32, -66}}, fontSize = 12, textString = "low voltage is detected in 
load1 and load4")}, coordinateSystem(initialScale = 0.1)),
          experiment(StopTime = 1));
      end Network;

      model DoubleNetwork "Same network with two LV feeders"
        extends Icons.myExample;
        Components.myLine line1(UNom = 20000, Imax = 160, R = 0.9, X = 0.3) annotation (
          Placement(visible = true, transformation(origin = {-30, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myTransformer tra2(UNomA = 20000, UNomB = 400, SNom = 250, R = 1e-9, X = 1e-9) annotation (
          Placement(visible = true, transformation(origin = {0, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load1(P = 36000 / sqrt(1.16), UNom = 400, Q = 0.4 * 36000 / sqrt(1.16)) annotation (
          Placement(visible = true, transformation(origin = {78, 96}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myTransformer tra1(UNomA = 63000, UNomB = 20000, SNom = 36000, R = 1e-9, X = 1e-9) annotation (
          Placement(visible = true, transformation(origin = {-62, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line2(UNom = 400, Imax = 140, R = 0.2, X = 0.2) annotation (
          Placement(visible = true, transformation(origin = {50, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load2(P = 36000 / sqrt(1.16), UNom = 400, Q = 0.4 * 36000 / sqrt(1.16)) annotation (
          Placement(visible = true, transformation(origin = {80, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line4(UNom = 400, Imax = 140, R = 0.2, X = 0.2) annotation (
          Placement(visible = true, transformation(origin = {50, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load3(P = 36000 / sqrt(1.16), UNom = 400, Q = 0.4 * 36000 / sqrt(1.16)) annotation (
          Placement(visible = true, transformation(origin = {80, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.mySource src(UNom = 63000) annotation (
          Placement(visible = true, transformation(origin = {-92, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line3(UNom = 400, Imax = 140, R = 0.2, X = 0.2) annotation (
          Placement(visible = true, transformation(origin = {50, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line5(UNom = 400, Imax = 140, R = 0.2, X = 0.2) annotation (
          Placement(visible = true, transformation(origin = {88, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load4(P = 36000 / sqrt(1.16), UNom = 400, Q = 0.4 * 36000 / sqrt(1.16)) annotation (
          Placement(visible = true, transformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load5(P = 36000 / sqrt(1.16), UNom = 400, Q = 0.4 * 36000 / sqrt(1.16)) annotation (
          Placement(visible = true, transformation(origin = {80, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line6(UNom = 400, Imax = 140, R = 0.2, X = 0.2) annotation (
          Placement(visible = true, transformation(origin = {52, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load6(P = 36000 / sqrt(1.16), UNom = 400, Q = 0.4 * 36000 / sqrt(1.16)) annotation (
          Placement(visible = true, transformation(origin = {82, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line7(UNom = 400, Imax = 140, R = 0.2, X = 0.2) annotation (
          Placement(visible = true, transformation(origin = {52, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load7(P = 36000 / sqrt(1.16), UNom = 400, Q = 0.4 * 36000 / sqrt(1.16)) annotation (
          Placement(visible = true, transformation(origin = {82, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line8(UNom = 400, Imax = 140, R = 0.2, X = 0.2) annotation (
          Placement(visible = true, transformation(origin = {52, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line9(UNom = 400, Imax = 140, R = 0.2, X = 0.2) annotation (
          Placement(visible = true, transformation(origin = {90, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load8(P = 36000 / sqrt(1.16), UNom = 400, Q = 0.4 * 36000 / sqrt(1.16)) annotation (
          Placement(visible = true, transformation(origin = {112, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(line1.terminalB, tra2.terminalA) annotation (
          Line(points = {{-20, 38}, {-10, 38}}));
        connect(line2.terminalB, load1.terminal) annotation (
          Line(points = {{60, 60}, {70, 60}, {70, 96.02}, {77.96, 96.02}}));
        connect(line4.terminalB, load3.terminal) annotation (
          Line(points = {{60, 16}, {70, 16}, {70, 16.02}, {79.96, 16.02}}));
        connect(src.terminal, tra1.terminalA) annotation (
          Line(points = {{-92, 38}, {-72, 38}}, color = {0, 0, 0}));
        connect(tra1.terminalB, line1.terminalA) annotation (
          Line(points = {{-52, 38}, {-40, 38}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line2.terminalA) annotation (
          Line(points = {{10, 38}, {24, 38}, {24, 60}, {40, 60}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line4.terminalA) annotation (
          Line(points = {{10, 38}, {24, 38}, {24, 16}, {40, 16}}, color = {0, 0, 0}));
        connect(load2.terminal, line3.terminalB) annotation (
          Line(points = {{79.96, 38.02}, {70, 38.02}, {70, 38}, {60, 38}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line3.terminalA) annotation (
          Line(points = {{10, 38}, {40, 38}}, color = {0, 0, 0}));
        connect(line2.terminalB, line5.terminalA) annotation (
          Line(points = {{60, 60}, {78, 60}}, color = {0, 0, 0}));
        connect(line5.terminalB, load4.terminal) annotation (
          Line(points = {{98, 60}, {102, 60}, {102, 60.02}, {109.96, 60.02}}, color = {0, 0, 0}));
        connect(line6.terminalB, load5.terminal) annotation (
          Line(points = {{62, -40}, {72, -40}, {72, -3.98}, {79.96, -3.98}}));
        connect(line7.terminalB, load7.terminal) annotation (
          Line(points = {{62, -84}, {72, -84}, {72, -83.98}, {81.96, -83.98}}));
        connect(load6.terminal, line8.terminalB) annotation (
          Line(points = {{81.96, -61.98}, {72, -61.98}, {72, -62}, {62, -62}}, color = {0, 0, 0}));
        connect(line6.terminalB, line9.terminalA) annotation (
          Line(points = {{62, -40}, {80, -40}}, color = {0, 0, 0}));
        connect(line9.terminalB, load8.terminal) annotation (
          Line(points = {{100, -40}, {104, -40}, {104, -39.98}, {111.96, -39.98}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line6.terminalA) annotation (
          Line(points = {{10, 38}, {24, 38}, {24, -40}, {42, -40}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line8.terminalA) annotation (
          Line(points = {{10, 38}, {24, 38}, {24, -62}, {42, -62}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line7.terminalA) annotation (
          Line(points = {{10, 38}, {24, 38}, {24, -84}, {42, -84}}, color = {0, 0, 0}));
        annotation (
          Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-114, 64}, {-70, 50}}, textStyle = {TextStyle.Bold}, fontSize = 10, textString = "RTE"), Text(lineColor = {28, 108, 200}, extent = {{-98, 4}, {-24, -40}}, fontSize = 12, textString = "same network with two LV feeders
low voltage is detected in 
load1, load4, load5 and load8
nominal power exceed detected in tra2"), Text(lineColor = {28, 108, 200}, extent = {{-24, 98}, {52, 76}}, fontSize = 10, textStyle = {TextStyle.Bold}, textString = "All residential loads
36 kVA (Q=0.4*P)")}, coordinateSystem(initialScale = 0.1)),
          experiment(StopTime = 1));
      end DoubleNetwork;
    end MediumNetworks;

    model FourVariableLoadsOneLVFeeder "Realistic case with three LV feeders and four variable loads"
      extends Icons.myExample;
      Components.mySource src(UNom = 63000) annotation (
        Placement(visible = true, transformation(origin = {-118, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine LVln2(UNom = 400, R = 0.2, X = 0.2, Imax = 45) annotation (
        Placement(visible = true, transformation(origin = {18, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myVariableTransformer HVMV(UNomA = 63000, UNomB = 20000, SNom = 100000, R = 1e-6) annotation (
        Placement(transformation(extent = {{-102, -10}, {-82, 10}})));
      Components.myTransformer MVLV(UNomA = 20000, UNomB = 400, SNom = 250, R = 1e-9) annotation (
        Placement(transformation(extent = {{-42, -10}, {-22, 10}})));
      Components.myLine MVline(UNom = 20000, R = 0.9, X = 0.3) annotation (
        Placement(transformation(extent = {{-72, -10}, {-52, 10}})));
      Components.myLine LVln1(UNom = 400, R = 0.2, X = 0.2, Imax = 45) annotation (
        Placement(transformation(extent = {{-6, 30}, {14, 50}})));
      Components.myLine LVln11(UNom = 400, R = 0.2, X = 0.2, Imax = 45) annotation (
        Placement(transformation(extent = {{22, 30}, {42, 50}})));
      Components.myLine LVln3(UNom = 400, R = 0.2, X = 0.2, Imax = 45) annotation (
        Placement(transformation(extent = {{8, -50}, {28, -30}})));
      Components.myVariableLoad varLoad1(UNom = 400) annotation (
        Placement(transformation(extent = {{44, 70}, {64, 90}})));
      Components.myVariableLoad varLoad2(UNom = 400) annotation (
        Placement(transformation(extent = {{42, 30}, {62, 50}})));
      Components.myVariableLoad varLoad3(UNom = 400) annotation (
        Placement(transformation(extent = {{42, -10}, {62, 10}})));
      Components.myVariableLoad varLoad4(UNom = 400) annotation (
        Placement(transformation(extent = {{40, -50}, {60, -30}})));
      Modelica.Blocks.Tables.CombiTable1Ds load(tableOnFile = true, tableName = "load", columns = 2:3, fileName = Modelica.Utilities.Files.loadResource(loadFile)) annotation (
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {140, 0})));
      Modelica.Blocks.Sources.RealExpression t(y = time) annotation (
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {174, 0})));
      Modelica.Blocks.Math.Gain gainP(k = 7000) annotation (
        Placement(transformation(extent = {{-6, -6}, {6, 6}}, rotation = 180, origin = {100, 40})));
      Modelica.Blocks.Math.Gain gainQ(k = 7000) annotation (
        Placement(transformation(extent = {{-6, -6}, {6, 6}}, rotation = 180, origin = {102, -28})));
      parameter String loadFile = "modelica://PowerSysPro/Resources/Files/variablePQLoad.csv" "External file for PQ variations";
    equation
      connect(MVLV.terminalB, LVln3.terminalA) annotation (
        Line(points = {{-22, 0}, {-14, 0}, {-14, -40}, {8, -40}}, color = {0, 0, 0}));
      connect(LVln2.terminalA, MVLV.terminalB) annotation (
        Line(points = {{8, 0}, {-22, 0}}, color = {0, 0, 0}));
      connect(MVLV.terminalB, LVln1.terminalA) annotation (
        Line(points = {{-22, 0}, {-14, 0}, {-14, 40}, {-6, 40}}, color = {0, 0, 0}));
      connect(LVln1.terminalB, LVln11.terminalA) annotation (
        Line(points = {{14, 40}, {22, 40}}, color = {0, 0, 0}));
      connect(HVMV.terminalA, src.terminal) annotation (
        Line(points = {{-102, 0}, {-118, 0}}, color = {0, 0, 0}));
      connect(MVline.terminalA, HVMV.terminalB) annotation (
        Line(points = {{-72, 0}, {-82, 0}}, color = {0, 0, 0}));
      connect(MVLV.terminalA, MVline.terminalB) annotation (
        Line(points = {{-42, 0}, {-52, 0}}, color = {0, 0, 0}));
      connect(LVln11.terminalB, varLoad2.terminal) annotation (
        Line(points = {{42, 40}, {48, 40}, {48, 40.02}, {51.96, 40.02}}, color = {0, 0, 0}));
      connect(LVln2.terminalB, varLoad3.terminal) annotation (
        Line(points = {{28, 0}, {32, 0}, {32, 0.02}, {51.96, 0.02}}, color = {0, 0, 0}));
      connect(LVln3.terminalB, varLoad4.terminal) annotation (
        Line(points = {{28, -40}, {32, -40}, {32, -39.98}, {49.96, -39.98}}, color = {0, 0, 0}));
      connect(t.y, load.u) annotation (
        Line(points = {{163, 0}, {152, 0}}, color = {0, 0, 127}));
      connect(load.y[1], gainP.u) annotation (
        Line(points = {{129, 8.88178e-16}, {122, 8.88178e-16}, {122, 40}, {107.2, 40}}, color = {0, 0, 127}));
      connect(load.y[2], gainQ.u) annotation (
        Line(points = {{129, 0}, {122, 0}, {122, -28}, {109.2, -28}}, color = {0, 0, 127}));
      connect(gainP.y, varLoad1.PInput) annotation (
        Line(points = {{93.4, 40}, {84, 40}, {84, 60}, {51.46, 60}, {51.46, 71.55}}, color = {0, 0, 127}));
      connect(gainP.y, varLoad2.PInput) annotation (
        Line(points = {{93.4, 40}, {84, 40}, {84, 20}, {49.46, 20}, {49.46, 31.55}}, color = {0, 0, 127}));
      connect(gainP.y, varLoad3.PInput) annotation (
        Line(points = {{93.4, 40}, {84, 40}, {84, -20}, {49.46, -20}, {49.46, -8.45}}, color = {0, 0, 127}));
      connect(gainP.y, varLoad4.PInput) annotation (
        Line(points = {{93.4, 40}, {84, 40}, {84, -60}, {47.46, -60}, {47.46, -48.45}}, color = {0, 0, 127}));
      connect(gainQ.y, varLoad1.QInput) annotation (
        Line(points = {{95.4, -28}, {72, -28}, {72, 64}, {56.565, 64}, {56.565, 71.565}}, color = {0, 0, 127}));
      connect(gainQ.y, varLoad2.QInput) annotation (
        Line(points = {{95.4, -28}, {72, -28}, {72, 26}, {54.565, 26}, {54.565, 31.565}}, color = {0, 0, 127}));
      connect(gainQ.y, varLoad3.QInput) annotation (
        Line(points = {{95.4, -28}, {54.565, -28}, {54.565, -8.435}}, color = {0, 0, 127}));
      connect(gainQ.y, varLoad4.QInput) annotation (
        Line(points = {{95.4, -28}, {72, -28}, {72, -56}, {52.565, -56}, {52.565, -48.435}}, color = {0, 0, 127}));
      connect(LVln11.terminalB, varLoad1.terminal) annotation (
        Line(points = {{42, 40}, {42, 80.02}, {53.96, 80.02}}, color = {0, 0, 0}));
      annotation (
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics={  Text(lineColor = {28, 108, 200}, extent = {{-164, 80}, {-12, 58}}, fontSize = 12, textString = "all loads are variable
simulation is done for 1-year duration")}),
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}})),
        experiment(StopTime = 31532400));
    end FourVariableLoadsOneLVFeeder;

    model LargerNetwork "This model has been partly generated by a Python tool"
      extends Icons.myExample;
      constant Real startP = 16000;
      constant Real startQ = 6400;
      Components.mySource src(UNom = 225000) annotation (
        Placement(transformation(extent = {{-270, 94}, {-250, 114}})));
      Components.myTransformer hvmv(UNomA = 225000, UNomB = 20000, R = 7.250000e-01, X = 2 * 3.1456 * 50 * 1.151031e-01) annotation (
        Placement(transformation(extent = {{-210, 94}, {-190, 114}})));
      Components.myTransformer mvlv11(UNomA = 20000, UNomB = 400, R = 6.679134e+00, X = 2 * 3.1456 * 50 * 6.764085e-02) annotation (
        Placement(transformation(extent = {{26, 46}, {46, 66}})));
      Components.myTransformer mvlv24(UNomA = 20000, UNomB = 400, R = 1.220206e+01, X = 2 * 3.1456 * 50 * 1.069521e-01) annotation (
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 0, origin = {36, -58})));
      Components.myLine ln1(UNom = 20000, l = 500, R = 2.000000e-04, X = 1.000000e-04, B = 5.620000e-08) annotation (
        Placement(transformation(extent = {{-80, 52}, {-60, 72}})));
      Components.myLine ln3(UNom = 20000, l = 1000, R = 1.130000e-04, X = 3.900000e-04, B = 1.570000e-09) annotation (
        Placement(transformation(extent = {{-242, 94}, {-222, 114}})));
      Components.myLine ln677(UNom = 20000, l = 4.314298e+03, R = 2.000000e-04, X = 1.000000e-04, B = 5.620000e-08) annotation (
        Placement(transformation(extent = {{-170, 30}, {-150, 50}})));
      Components.myLine ln714(UNom = 20000, l = 2.163435e+02, R = 2.000000e-04, X = 1.000000e-04, B = 5.620000e-08) annotation (
        Placement(transformation(extent = {{-142, -30}, {-122, -10}})));
      Components.myLine ln715(UNom = 20000, l = 1.029283e+02, R = 2.000000e-04, X = 1.000000e-04, B = 5.620000e-08) annotation (
        Placement(transformation(extent = {{-132, 26}, {-112, 46}})));
      Components.myLine ln716(UNom = 20000, l = 2.065060e+02, R = 2.000000e-04, X = 1.000000e-04, B = 5.620000e-08) annotation (
        Placement(transformation(extent = {{-100, -76}, {-80, -56}})));
      Components.myLine ln717(UNom = 20000, l = 1.040661e+02, R = 2.000000e-04, X = 1.000000e-04, B = 5.620000e-08) annotation (
        Placement(transformation(extent = {{-102, -52}, {-82, -32}})));
      Components.myLine ln718(UNom = 20000, l = 6.536973e+01, R = 2.000000e-04, X = 1.000000e-04, B = 5.620000e-08) annotation (
        Placement(transformation(extent = {{-100, 2}, {-80, 22}})));
      Components.myLine ln719(UNom = 20000, l = 7.358724e+01, R = 2.000000e-04, X = 1.000000e-04, B = 5.620000e-08) annotation (
        Placement(transformation(extent = {{-70, -102}, {-50, -82}})));
      Components.myLine ln676(UNom = 20000, l = 4.122333e+03, R = 1.354310e-04, X = 1.854310e-04, B = 5.000000e-08) annotation (
        Placement(transformation(extent = {{-52, 94}, {-32, 114}})));
      Components.myLine ln702(UNom = 20000, l = 2.030604e+02, R = 2.000000e-04, X = 1.000000e-04, B = 5.620000e-08) annotation (
        Placement(transformation(extent = {{-32, -36}, {-12, -16}})));
      Components.myLine ln703(UNom = 20000, l = 4.903295e+02, R = 2.000000e-04, X = 1.000000e-04, B = 5.620000e-08) annotation (
        Placement(transformation(extent = {{-28, 70}, {-8, 90}})));
      Components.myLine ln712(UNom = 20000, l = 1.313188e+02, R = 2.000000e-04, X = 1.000000e-04, B = 5.620000e-08) annotation (
        Placement(transformation(extent = {{-4, -68}, {16, -48}})));
      Components.myLine ln713(UNom = 20000, l = 8.874849e+01, R = 2.000000e-04, X = 1.000000e-04, B = 5.620000e-08) annotation (
        Placement(transformation(extent = {{-4, 46}, {16, 66}})));
      Components.myLine ln720(UNom = 20000, l = 1.787394e+02, R = 2.000000e-04, X = 1.000000e-04, B = 5.620000e-08) annotation (
        Placement(transformation(extent = {{20, -94}, {40, -74}})));
      Components.myLine ln769(UNom = 400, l = 1.372877e+02, R = 1.354310e-04, X = 1.854310e-04, B = 5.000000e-08) annotation (
        Placement(transformation(extent = {{58, -16}, {78, 4}})));
      Components.myLine ln770(UNom = 400, l = 1.535058e+02, R = 1.354310e-04, X = 1.854310e-04, B = 5.000000e-08) annotation (
        Placement(transformation(extent = {{60, 24}, {80, 44}})));
      Components.myLine ln771(UNom = 400, l = 2.126950e+02, R = 1.354310e-04, X = 1.854310e-04, B = 5.000000e-08) annotation (
        Placement(transformation(extent = {{60, 46}, {80, 66}})));
      Components.myLine ln772(UNom = 400, l = 3.493784e+01, R = 3.443400e-04, X = 3.943400e-04, B = 5.000000e-08) annotation (
        Placement(transformation(extent = {{82, -42}, {102, -22}})));
      Components.myLine ln773(UNom = 400, l = 1.811677e+01, R = 3.443400e-04, X = 3.943400e-04, B = 5.000000e-08) annotation (
        Placement(transformation(extent = {{116, -66}, {136, -46}})));
      Components.myLine ln774(UNom = 400, l = 3.051450e+01, R = 3.443400e-04, X = 3.943400e-04, B = 5.000000e-08) annotation (
        Placement(transformation(extent = {{86, 4}, {106, 24}})));
      Components.myLine ln775(UNom = 400, l = 5.738786e+01, R = 1.354310e-04, X = 1.854310e-04, B = 5.000000e-08) annotation (
        Placement(transformation(extent = {{54, -68}, {74, -48}})));
      Components.myLine ln776(UNom = 400, l = 1.100241e+02, R = 2.157870e-04, X = 2.657870e-04, B = 5.000000e-08) annotation (
        Placement(transformation(extent = {{82, -90}, {102, -70}})));
      Components.myLine ln777(UNom = 400, l = 1.300031e+02, R = 3.443400e-04, X = 3.943400e-04, B = 5.000000e-08) annotation (
        Placement(transformation(extent = {{82, -110}, {102, -90}})));
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361280_Pload(start = startP);
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361280_Qload(start = startQ);
      Components.myVariableLoad b61280(UNom = 20000) annotation (
        Placement(transformation(extent = {{-140, 56}, {-120, 76}})));
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000005011458_Pload(start = startP);
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000005011458_Qload(start = startQ);
      Components.myVariableLoad b11458(UNom = 20000) annotation (
        Placement(transformation(extent = {{-106, -30}, {-86, -10}})));
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361256_Pload(start = startP);
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361256_Qload(start = startQ);
      Components.myVariableLoad b61256(UNom = 20000) annotation (
        Placement(transformation(extent = {{-110, 26}, {-90, 46}})));
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000005011480_Pload(start = startP);
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000005011480_Qload(start = startQ);
      Components.myVariableLoad b11480(UNom = 20000) annotation (
        Placement(transformation(extent = {{-82, 2}, {-62, 22}})));
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361232_Pload(start = startP);
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361232_Qload(start = startQ);
      Components.myVariableLoad b61232(UNom = 20000) annotation (
        Placement(transformation(extent = {{-76, -76}, {-56, -56}})));
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361121_Pload(start = startP);
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361121_Qload(start = startQ);
      Components.myVariableLoad b61121(UNom = 20000) annotation (
        Placement(transformation(extent = {{-76, -52}, {-56, -32}})));
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000356487694_Pload(start = startP);
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000356487694_Qload(start = startQ);
      Components.myVariableLoad b87694(UNom = 20000) annotation (
        Placement(transformation(extent = {{-50, -102}, {-30, -82}})));
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000005011465_Pload(start = startP);
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000005011465_Qload(start = startQ);
      Components.myVariableLoad b11465(UNom = 20000) annotation (
        Placement(transformation(extent = {{-30, 94}, {-10, 114}})));
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361260_Pload(start = startP);
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361260_Qload(start = startQ);
      Components.myVariableLoad b61260(UNom = 20000) annotation (
        Placement(transformation(extent = {{-4, 70}, {16, 90}})));
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361254_Pload(start = startP);
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361254_Qload(start = startQ);
      Components.myVariableLoad b61254(UNom = 400) annotation (
        Placement(transformation(extent = {{82, -16}, {102, 4}})));
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000005011407_Pload(start = startP);
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000005011407_Qload(start = startQ);
      Components.myVariableLoad b11407(UNom = 400) annotation (
        Placement(transformation(extent = {{108, -42}, {128, -22}})));
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000005011403_Pload(start = startP);
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000005011403_Qload(start = startQ);
      Components.myVariableLoad b11403(UNom = 400) annotation (
        Placement(transformation(extent = {{136, -66}, {156, -46}})));
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361227_Pload(start = startP);
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361227_Qload(start = startQ);
      Components.myVariableLoad b61227(UNom = 20000) annotation (
        Placement(transformation(extent = {{-10, -36}, {10, -16}})));
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361089_Pload(start = startP);
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361089_Qload(start = startQ);
      Components.myVariableLoad b61089(UNom = 20000) annotation (
        Placement(transformation(extent = {{38, -94}, {58, -74}})));
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361274_Pload(start = startP);
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361274_Qload(start = startQ);
      Components.myVariableLoad b61274(UNom = 400) annotation (
        Placement(transformation(extent = {{84, 24}, {104, 44}})));
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361273_Pload(start = startP);
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361273_Qload(start = startQ);
      Components.myVariableLoad b61273(UNom = 400) annotation (
        Placement(transformation(extent = {{100, 4}, {120, 24}})));
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361275_Pload(start = startP);
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361275_Qload(start = startQ);
      Components.myVariableLoad b61275(UNom = 400) annotation (
        Placement(transformation(extent = {{84, 46}, {104, 66}})));
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000320456131_Pload(start = startP);
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000320456131_Qload(start = startQ);
      Components.myVariableLoad b56131(UNom = 400) annotation (
        Placement(transformation(extent = {{76, -68}, {96, -48}})));
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000356487656_Pload(start = startP);
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000356487656_Qload(start = startQ);
      Components.myVariableLoad b87656(UNom = 400) annotation (
        Placement(transformation(extent = {{100, -90}, {120, -70}})));
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361106_Pload(start = startP);
      Modelica.Blocks.Interfaces.RealInput BUILDING0000000322361106_Qload(start = startQ);
      Components.myVariableLoad b61106(UNom = 400) annotation (
        Placement(transformation(extent = {{100, -110}, {120, -90}})));
    equation
      connect(b61280.PInput, BUILDING0000000322361280_Pload);
      connect(b61280.QInput, BUILDING0000000322361280_Qload);
      connect(b11458.PInput, BUILDING0000000005011458_Pload);
      connect(b11458.QInput, BUILDING0000000005011458_Qload);
      connect(b61256.PInput, BUILDING0000000322361256_Pload);
      connect(b61256.QInput, BUILDING0000000322361256_Qload);
      connect(b11480.PInput, BUILDING0000000005011480_Pload);
      connect(b11480.QInput, BUILDING0000000005011480_Qload);
      connect(b61232.PInput, BUILDING0000000322361232_Pload);
      connect(b61232.QInput, BUILDING0000000322361232_Qload);
      connect(b61121.PInput, BUILDING0000000322361121_Pload);
      connect(b61121.QInput, BUILDING0000000322361121_Qload);
      connect(b87694.PInput, BUILDING0000000356487694_Pload);
      connect(b87694.QInput, BUILDING0000000356487694_Qload);
      connect(b11465.PInput, BUILDING0000000005011465_Pload);
      connect(b11465.QInput, BUILDING0000000005011465_Qload);
      connect(b61260.PInput, BUILDING0000000322361260_Pload);
      connect(b61260.QInput, BUILDING0000000322361260_Qload);
      connect(b61254.PInput, BUILDING0000000322361254_Pload);
      connect(b61254.QInput, BUILDING0000000322361254_Qload);
      connect(b11407.PInput, BUILDING0000000005011407_Pload);
      connect(b11407.QInput, BUILDING0000000005011407_Qload);
      connect(b11403.PInput, BUILDING0000000005011403_Pload);
      connect(b11403.QInput, BUILDING0000000005011403_Qload);
      connect(b61227.PInput, BUILDING0000000322361227_Pload);
      connect(b61227.QInput, BUILDING0000000322361227_Qload);
      connect(b61089.PInput, BUILDING0000000322361089_Pload);
      connect(b61089.QInput, BUILDING0000000322361089_Qload);
      connect(b61274.PInput, BUILDING0000000322361274_Pload);
      connect(b61274.QInput, BUILDING0000000322361274_Qload);
      connect(b61273.PInput, BUILDING0000000322361273_Pload);
      connect(b61273.QInput, BUILDING0000000322361273_Qload);
      connect(b61275.PInput, BUILDING0000000322361275_Pload);
      connect(b61275.QInput, BUILDING0000000322361275_Qload);
      connect(b56131.PInput, BUILDING0000000320456131_Pload);
      connect(b56131.QInput, BUILDING0000000320456131_Qload);
      connect(b87656.PInput, BUILDING0000000356487656_Pload);
      connect(b87656.QInput, BUILDING0000000356487656_Qload);
      connect(b61106.PInput, BUILDING0000000322361106_Pload);
      connect(b61106.QInput, BUILDING0000000322361106_Qload);
      connect(src.terminal, ln3.terminalA) annotation (
        Line(points = {{-260, 104}, {-242, 104}}, color = {0, 0, 0}));
      connect(ln3.terminalB, hvmv.terminalA) annotation (
        Line(points = {{-222, 104}, {-210, 104}}, color = {0, 0, 0}));
      connect(hvmv.terminalB, ln676.terminalA) annotation (
        Line(points = {{-190, 104}, {-52, 104}}, color = {0, 0, 0}));
      connect(ln676.terminalB, b11465.terminal) annotation (
        Line(points = {{-32, 104}, {-28, 104}, {-28, 104.02}, {-20.04, 104.02}}, color = {0, 0, 0}));
      connect(ln703.terminalB, b61260.terminal) annotation (
        Line(points = {{-8, 80}, {-2, 80}, {-2, 80.02}, {5.96, 80.02}}, color = {0, 0, 0}));
      connect(ln676.terminalB, ln703.terminalA) annotation (
        Line(points = {{-32, 104}, {-32, 80}, {-28, 80}}, color = {0, 0, 0}));
      connect(ln713.terminalB, mvlv11.terminalA) annotation (
        Line(points = {{16, 56}, {26, 56}}, color = {0, 0, 0}));
      connect(ln771.terminalB, b61275.terminal) annotation (
        Line(points = {{80, 56}, {86, 56}, {86, 56.02}, {93.96, 56.02}}, color = {0, 0, 0}));
      connect(ln770.terminalB, b61274.terminal) annotation (
        Line(points = {{80, 34}, {93.96, 34}, {93.96, 34.02}}, color = {0, 0, 0}));
      connect(ln774.terminalB, b61273.terminal) annotation (
        Line(points = {{106, 14}, {108, 14}, {108, 14.02}, {109.96, 14.02}}, color = {0, 0, 0}));
      connect(ln769.terminalB, b61254.terminal) annotation (
        Line(points = {{78, -6}, {84, -6}, {84, -5.98}, {91.96, -5.98}}, color = {0, 0, 0}));
      connect(mvlv11.terminalB, ln769.terminalA) annotation (
        Line(points = {{46, 56}, {46, -8}, {58, -8}, {58, -6}}, color = {0, 0, 0}));
      connect(mvlv11.terminalB, ln771.terminalA) annotation (
        Line(points = {{46, 56}, {60, 56}}, color = {0, 0, 0}));
      connect(mvlv11.terminalB, ln770.terminalA) annotation (
        Line(points = {{46, 56}, {52, 56}, {52, 34}, {60, 34}}, color = {0, 0, 0}));
      connect(ln769.terminalB, ln772.terminalA) annotation (
        Line(points = {{78, -6}, {78, -32}, {82, -32}}, color = {0, 0, 0}));
      connect(ln772.terminalB, b11407.terminal) annotation (
        Line(points = {{102, -32}, {112, -32}, {112, -31.98}, {117.96, -31.98}}, color = {0, 0, 0}));
      connect(ln772.terminalB, ln773.terminalA) annotation (
        Line(points = {{102, -32}, {102, -56}, {116, -56}}, color = {0, 0, 0}));
      connect(ln773.terminalB, b11403.terminal) annotation (
        Line(points = {{136, -56}, {140, -56}, {140, -55.98}, {145.96, -55.98}}, color = {0, 0, 0}));
      connect(ln676.terminalB, ln702.terminalA) annotation (
        Line(points = {{-32, 104}, {-32, -26}}, color = {0, 0, 0}));
      connect(ln703.terminalB, ln713.terminalA) annotation (
        Line(points = {{-8, 80}, {-8, 56}, {-4, 56}}, color = {0, 0, 0}));
      connect(ln702.terminalB, b61227.terminal) annotation (
        Line(points = {{-12, -26}, {-4, -26}, {-4, -25.98}, {-0.04, -25.98}}, color = {0, 0, 0}));
      connect(ln702.terminalB, ln712.terminalA) annotation (
        Line(points = {{-12, -26}, {-8, -26}, {-8, -58}, {-4, -58}}, color = {0, 0, 0}));
      connect(ln712.terminalB, mvlv24.terminalA) annotation (
        Line(points = {{16, -58}, {26, -58}}, color = {0, 0, 0}));
      connect(mvlv24.terminalB, ln775.terminalA) annotation (
        Line(points = {{46, -58}, {54, -58}}, color = {0, 0, 0}));
      connect(ln775.terminalB, b56131.terminal) annotation (
        Line(points = {{74, -58}, {76, -58}, {76, -57.98}, {85.96, -57.98}}, color = {0, 0, 0}));
      connect(ln775.terminalB, ln776.terminalA) annotation (
        Line(points = {{74, -58}, {74, -80}, {82, -80}}, color = {0, 0, 0}));
      connect(ln776.terminalB, b87656.terminal) annotation (
        Line(points = {{102, -80}, {102, -79.98}, {109.96, -79.98}}, color = {0, 0, 0}));
      connect(ln775.terminalB, ln777.terminalA) annotation (
        Line(points = {{74, -58}, {74, -100}, {82, -100}}, color = {0, 0, 0}));
      connect(ln777.terminalB, b61106.terminal) annotation (
        Line(points = {{102, -100}, {104, -100}, {104, -99.98}, {109.96, -99.98}}, color = {0, 0, 0}));
      connect(ln712.terminalB, ln720.terminalA) annotation (
        Line(points = {{16, -58}, {16, -84}, {20, -84}}, color = {0, 0, 0}));
      connect(ln720.terminalB, b61089.terminal) annotation (
        Line(points = {{40, -84}, {44, -84}, {44, -83.98}, {47.96, -83.98}}, color = {0, 0, 0}));
      connect(ln677.terminalB, b61280.terminal) annotation (
        Line(points = {{-150, 40}, {-138, 40}, {-138, 66.02}, {-130.04, 66.02}}, color = {0, 0, 0}));
      connect(ln677.terminalB, ln715.terminalA) annotation (
        Line(points = {{-150, 40}, {-140, 40}, {-140, 36}, {-132, 36}}, color = {0, 0, 0}));
      connect(ln715.terminalB, b61256.terminal) annotation (
        Line(points = {{-112, 36}, {-106, 36}, {-106, 36.02}, {-100.04, 36.02}}, color = {0, 0, 0}));
      connect(ln718.terminalB, b11480.terminal) annotation (
        Line(points = {{-80, 12}, {-76, 12}, {-76, 12.02}, {-72.04, 12.02}}, color = {0, 0, 0}));
      connect(ln677.terminalB, ln714.terminalA) annotation (
        Line(points = {{-150, 40}, {-150, -20}, {-142, -20}}, color = {0, 0, 0}));
      connect(ln717.terminalB, b61121.terminal) annotation (
        Line(points = {{-82, -42}, {-74, -42}, {-74, -41.98}, {-66.04, -41.98}}, color = {0, 0, 0}));
      connect(ln716.terminalB, ln719.terminalA) annotation (
        Line(points = {{-80, -66}, {-80, -92}, {-70, -92}}, color = {0, 0, 0}));
      connect(ln719.terminalB, b87694.terminal) annotation (
        Line(points = {{-50, -92}, {-50, -91.98}, {-40.04, -91.98}}, color = {0, 0, 0}));
      connect(ln716.terminalB, b61232.terminal) annotation (
        Line(points = {{-80, -66}, {-72, -66}, {-72, -65.98}, {-66.04, -65.98}}, color = {0, 0, 0}));
      connect(ln714.terminalB, ln716.terminalA) annotation (
        Line(points = {{-122, -20}, {-122, -66}, {-100, -66}}, color = {0, 0, 0}));
      connect(ln714.terminalB, b11458.terminal) annotation (
        Line(points = {{-122, -20}, {-122, -19.98}, {-96.04, -19.98}}, color = {0, 0, 0}));
      connect(ln714.terminalB, ln717.terminalA) annotation (
        Line(points = {{-122, -20}, {-122, -42}, {-102, -42}}, color = {0, 0, 0}));
      connect(ln770.terminalB, ln774.terminalA) annotation (
        Line(points = {{80, 34}, {80, 14}, {86, 14}}, color = {0, 0, 0}));
      connect(ln715.terminalB, ln1.terminalA) annotation (
        Line(points = {{-112, 36}, {-106, 36}, {-106, 62}, {-80, 62}}, color = {0, 0, 0}));
      connect(ln715.terminalB, ln718.terminalA) annotation (
        Line(points = {{-112, 36}, {-106, 36}, {-106, 12}, {-100, 12}}, color = {0, 0, 0}));
      connect(hvmv.terminalB, ln677.terminalA) annotation (
        Line(points = {{-190, 104}, {-178, 104}, {-178, 40}, {-170, 40}}, color = {0, 0, 0}));
      connect(ln712.terminalB, ln1.terminalB) annotation (
        Line(points = {{16, -58}, {16, -72}, {-44, -72}, {-44, 62}, {-60, 62}}, color = {0, 0, 0}));
      annotation (
        experiment(StopTime = 1));
    end LargerNetwork;

    model DegradedModeForLoads "Testing the degraded mode for variable loads"
      extends Icons.myExample;
      Components.mySource src(UNom = 400) annotation (
        Placement(visible = true, transformation(origin = {-48, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line(UNom = 400, R = 1, X = 0) annotation (
        Placement(visible = true, transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myVariableLoad load(UNom = 400, switchToImpedanceMode = true) annotation (
        Placement(visible = true, transformation(origin = {50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Real MaxP = 40000;
      Real MaxQ = 0;
      Types.myApparentPower Sreduced = load.S;
      Types.myApparentPower Srequired = sqrt(load.PInput ^ 2 + load.QInput ^ 2);
      Real C;
    equation
      if time < 200 then
        C = time / 100;
      else
        C = (400 - time) / 100;
      end if;
      load.PInput = C * MaxP;
      load.QInput = C * MaxQ;
      connect(src.terminal, line.terminalA) annotation (
        Line(points = {{-48, 30}, {-10, 30}}));
      connect(line.terminalB, load.terminal) annotation (
        Line(points = {{10, 30}, {30, 30}, {30, 30.02}, {49.96, 30.02}}));
      annotation (
        Diagram(coordinateSystem(initialScale = 0.1), graphics={  Text(lineColor = {28, 108, 200}, extent = {{-206, -2}, {198, -66}}, fontSize = 12, textString = "The load switches to a degraded
mode at time 36 s (when voltage is too low)
and switches again to a normal mode
at time 364 s (when voltage is normal again)
The apparent power of the load is reduced
and can be compared to the required one")}),
        experiment(StopTime = 400));
    end DegradedModeForLoads;
    annotation (
      Documentation(info = "<html>
    <p>This package exhibits some complete cases in the spirit of electrical distribution grids.</p>
    <p>The last one is generated by an external Python code without any graphical representation.</p></body></html>"));
  end Examples;

  package Information "Information for users"
    extends Icons.myInformation;

    package Overview "Overview of the PowerSysPro library"
      extends Icons.myReleaseNotes;
      annotation (
        preferredView = "info",
        Documentation(info = "<html><head></head><body>
    <p>PowerSysPro library is a Modelica library for the load flow calculation of Distribution networks.</p>
    <p>All components are dramatically simplified to allow the modelling of medium and large large-scale systems.</p>
<p>For very huge grids, causal connectors are proposed in order to split the whole model in parts that can be exported as FMUs and co-simulated in tools like the open-source  <a href=\"https://bitbucket.org/simulage/daccosim/wiki/Home\">DACCOSIM NG</a> .</p>
</body></html>"));
    end Overview;

    package Contact "EDF contact for the PowerSysPro library"
      extends Icons.myContact;
      annotation (
        Documentation(info = "<html>
    <p><head></head><body>PowerSysPro library has been developed and is maintained by&nbsp;<a href=\"https://www.edf.fr/en/the-edf-group/who-we-are/activities/research-and-development\">EDF R&D</a> to promote the use of the Modelica language for Distribution networks with components modeled in an open-source approach.</p>
    <p>In case of questions or issues, you can send an e-mail to&nbsp;<a href=\"mailto:jean-philippe.tavella@edf.fr\">PowerSysPro contact</a>. </p></div></div></body></html>"));
    end Contact;
    annotation (
      preferredView = "info",
      Documentation(info = "<html><head></head><body>
    <p>PowerSysPro is a Modelica library dedicated to the load flow calculation of Distribution grids.</p>
    <p>PowerSysPro is being developed at EDF Lab Paris-Saclay.</p>
    <p></p><p>-------------------------------------------------------------------------------------------------------------</p>
    </body></html>"));
  end Information;
  annotation (
    version = "Version 2.1.2 February 18th, 2021",
    Documentation(info = "<html><head></head><body>
    <p>Copyright © 2020-2021, EDF.</p>
    <p>The use of the PowerSysPro library is granted by EDF under the provisions of the Modelica License 2. A copy of this license can be obtained&nbsp;<a href=\"http://www.modelica.org/licenses/ModelicaLicense2\">here</a>.</p>
    <p></p><p>-------------------------------------------------------------------------------------------------------------</p>
<p>For further technical information, see the <a href=\"modelica://PowerSysPro.Information\">Information</a>.</p>
</body></html>"),
    Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-174, 28}, {180, -28}}, fontSize = 14, textStyle = {TextStyle.Bold}, textString = "Open electrical library
developed at EDF Lab. Paris-Saclay")}),
    Icon(graphics={                                                                                                                                                                                                        Text(extent={{
              -208,70},{214,-60}},                                                                                                                                                                                                      lineColor=
              {28,108,200},                                                                                                                                                                                                        fontName=
              "Segoe Print",
          textString="PSP")}));
end PowerSysPro;

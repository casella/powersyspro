package PowerSysPro
  //Copyright 2020 EDF
  extends Icons.myPackage;
  import CM = PowerSysPro.Functions;

  package Components "Components for load flow calculations"
    extends PowerSysPro.Icons.myPackage;

    model mySource "Perfect voltage source"
      extends PartialModels.myPartialSource;
      extends Icons.mySource;
    equation
      terminal.v = CM.fromPolar(UNom / sqrt(3), theta);
      annotation (
        Icon(coordinateSystem(grid = {0.1, 0.1}, initialScale = 0.1)),
        Documentation(info = "<html>
    <p>This node prescribes the voltage magnitude and phase (default zero) for the source.</p>
</html>"));
    end mySource;

    model myEmergencySource "Emergency node"
      extends Icons.myPV;
      Interfaces.myAcausalTerminal terminal(v(re(start = UNom / sqrt(3)), im(start = 0))) "Terminal of the 1-port node" annotation (
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter Types.myVoltage UNom "Reference voltage of the node";
      parameter Types.myApparentPower Smax "Maximum  power the node can provide";
      parameter Types.myTime rampTime = 0 "Ramping up before the node is fully ready";
      Types.myCurrent I = CM.abs(terminal.i) "Current flowing the node";
      Types.myVoltage U = sqrt(3) * CM.abs(terminal.v) "Voltage at the node";
      Types.myApparentPower S = sqrt(3) * I * U "Apparent power flowing the node";
    //protected
      Real coeff;
    equation
      coeff = if rampTime == 0 then 1 else min(1, time / rampTime);
      when time > 0 then
        assert(S <= coeff * Smax, ">>> Required power is above the value that can be provided by " + getInstanceName());
      end when;
      terminal.v = CM.fromPolar(coeff * UNom / sqrt(3), 0);
      annotation (
        Icon(coordinateSystem(grid = {0.1, 0.1}, initialScale = 0.1, preserveAspectRatio = false), graphics={  Text(origin = {60.1, -63.1}, lineColor = {0, 0, 255}, extent = {{-124, 11}, {124, -11}}, textString = if OnePhaseLoad then "1" else "3", textStyle = {TextStyle.Bold})}),
        Documentation(info = "<html>
    <p>This node prescribes the constant active power <code>P</code> entering the PV bus.</p>
    </html>"),
        Diagram(coordinateSystem(preserveAspectRatio = false)));
    end myEmergencySource;

    model myBattery "Battery with inverter"
      extends PartialModels.myPartialSource;
      extends Icons.myBattery;
      replaceable parameter Batteries.DataSet.myDataSetNb1 dataSet "Set of parameters" annotation (
        Dialog(group = "Battery parameters"));
      parameter Integer Ns(min = 1) "Number of serial connected cells";
      parameter Integer Np(min = 1) "Number of parallel connected cells";
      parameter Real SOC_start "initial SOC" annotation (
        Dialog(group = "Charging/Discharging parameters"));
      parameter Real maxCharging = 0.8 "Maximum SOC in charging mode" annotation (
        Dialog(group = "Charging/Discharging parameters"));
      parameter Real minCharging = 0.2 "Minimum SOC in discharging mode" annotation (
        Dialog(group = "Charging/Discharging parameters"));
      parameter Types.myPerUnit efficiency = 0.9 "Converter/inverter efficiency";
      Batteries.Components.myIdealBattery battery(Ns = Ns, Np = Np, SOC_start = SOC_start, maxCharging = maxCharging, minCharging = minCharging, dataSet = dataSet) annotation (
        Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 270, origin = {84, 62})));
      Batteries.Basics.myGround ground annotation (
        Placement(transformation(extent = {{54, 22}, {74, 42}})));
      Batteries.Basics.mySignalCurrent inverter annotation (
        Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 90, origin = {30, 66})));
      Modelica.Blocks.Interfaces.BooleanInput ChargingMode "true means charging mode" annotation (
        Placement(transformation(extent = {{-120, 10}, {-80, 50}})));
      Types.myActivePower P "Power delivered by the battery";
    equation
      assert(SOC_start >= minCharging and SOC_start <= maxCharging, ">>> Incorrect value for SOC_start of " + getInstanceName());
      assert(battery.SOC >= minCharging, ">>> Battery " + getInstanceName() + " is empty");
      assert(battery.SOC <= maxCharging, ">>> Battery " + getInstanceName() + " is full", AssertionLevel.warning);
      // Power calculation of the battery with its inverter/rectifier component
      P = if battery.SOC >= minCharging and battery.SOC <= maxCharging then (battery.p.v - battery.n.v) / sqrt(2) * inverter.i else 0.0;
      if ChargingMode then
        3 * terminal.v * CM.conj(terminal.i) = Complex(-P, 0);
      else
        terminal.v = CM.fromPolar(UNom / sqrt(3), theta);
      end if;
      // Inverter and rectifier behaviours
      if ChargingMode then
        // In rectifier mode (ChargingMode==true), supplied current is constant
        efficiency * inverter.i = if battery.SOC <= maxCharging then -battery.dataSet.IchCoeff * battery.dataSet.QNom * Np else 0.0;
      else
        // In inverter mode (ChargingMode==false), the current is given by the downstream
        efficiency * inverter.i = if battery.SOC >= minCharging then 3 * I / sqrt(2) else 0.0;
      end if;
      connect(battery.n, ground.p) annotation (
        Line(points = {{84, 52}, {84, 42}, {64, 42}}, color = {0, 0, 255}));
      connect(ground.p, inverter.n) annotation (
        Line(points = {{64, 42}, {64, 56}, {30, 56}}, color = {0, 0, 255}));
      connect(inverter.p, battery.p) annotation (
        Line(points = {{30, 76}, {58, 76}, {58, 80}, {84, 80}, {84, 72}}, color = {0, 0, 255}));
      annotation (
        Icon(coordinateSystem(grid = {0.1, 0.1}, initialScale = 0.1)),
        Documentation(info = "<html>
    <p>This node prescribes the voltage magnitude and phase (default zero) for the source.</p>
</html>"),
        experiment(StopTime = 144000, __Dymola_Algorithm = "Dassl"),
        Diagram(graphics={  Text(extent = {{-90, -20}, {94, -74}}, textColor = {28, 108, 200}, fontSize = 12, textString = "Inverter/converter behaviour is
governed by the input ChargingMode
and a parameter for efficiency")}));
    end myBattery;

    model myLoad "Load with P/Q as fixed values"
      extends PartialModels.myPartialLoad;
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
      Modelica.Blocks.Interfaces.RealInput PInput(final unit = "W", displayUnit = "kW") "Active power consumed (>0) or provided (<0)" annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 0, origin = {-100, 40}), iconTransformation(extent = {{-15.3, -15.3}, {15.3, 15.3}}, rotation = 90, origin = {-25.4, -84.5})));
      Modelica.Blocks.Interfaces.RealInput QInput(final unit = "var", displayUnit = "kvar") "Reactive power inductive (>0) or capacitive (<0)" annotation (
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
      Interfaces.myAcausalTerminal terminal(v(re(start = UNom / sqrt(3)), im(start = 0))) "Terminal of the 1-port node" annotation (
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
      Interfaces.myAcausalTerminal terminalA(v(re(start = UNom / sqrt(3)), im(start = 0)), i(re(start = 0), im(start = 0))) "Terminal A of the 2-port node" annotation (
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Interfaces.myAcausalTerminal terminalB(v(re(start = UNom / sqrt(3)), im(start = 0)), i(re(start = 0), im(start = 0))) "Terminal B of the 2-port node" annotation (
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter Types.myVoltage UNom "Reference voltage of the line";
      parameter Types.myResistance R "Series resistance of phase conductor";
      parameter Types.myReactance X = 0 "Series reactance of phase conductor";
      parameter Types.myConductance G = 0 "Shunt conductance of phase conductor coupling";
      parameter Types.mySusceptance B = 0 "Shunt susceptance of phase conductor coupling";
      parameter Types.myPerUnit l = 1 "lineic coefficient";
      parameter Types.myCurrent Imax = 0 "Maximum admissible current. 0 means no control";
      Types.myCurrent IA = CM.abs(terminalA.i) "Current flowing at port A";
      Types.myVoltage UA = sqrt(3) * CM.abs(terminalA.v) "Voltage at port A";
      Types.myCurrent IB = CM.abs(terminalB.i) "Current flowing at port B";
      Types.myVoltage UB = sqrt(3) * CM.abs(terminalB.v) "Voltage at port B";
      Types.myApparentPower SA = sqrt(3) * IA * UA "Apparent power flowing at port A";
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

    model myLineWithFault "MV or LV line with constant impedance and fault at intermediate position"
      extends Icons.myLine;
      extends Icons.myTwoPortsAC;
      extends Icons.myFault;
      Interfaces.myAcausalTerminal terminalA(v(re(start = UNom / sqrt(3)), im(start = 0)), i(re(start = 0), im(start = 0))) "Terminal A of the 2-port node" annotation (
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Interfaces.myAcausalTerminal terminalB(v(re(start = UNom / sqrt(3)), im(start = 0)), i(re(start = 0), im(start = 0))) "Terminal B of the 2-port node" annotation (
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine lineA(UNom = UNom, R = faultLocationPu * l * R, X = faultLocationPu * l * X, B = faultLocationPu * l * B, G = faultLocationPu * l * G) annotation (
        Placement(visible = true, transformation(origin = {-40, 2.44249e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Components.myLine lineB(UNom = UNom, R = (1 - faultLocationPu) * l * R, X = (1 - faultLocationPu) * l * X, B = (1 - faultLocationPu) * l * B, G = (1 - faultLocationPu) * l * G) annotation (
        Placement(visible = true, transformation(origin = {40, 1.77636e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Components.PartialModels.myFault myFault(R = RFault, X = XFault, startTime = startTime, stopTime = stopTime) annotation (
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
      Types.myCurrent IA = CM.abs(terminalA.i) "Current flowing at port A";
      Types.myVoltage UA = sqrt(3) * CM.abs(terminalA.v) "Voltage at port A";
      Types.myCurrent IB = CM.abs(terminalB.i) "Current flowing at port B";
      Types.myVoltage UB = sqrt(3) * CM.abs(terminalB.v) "Voltage at port B";
      Types.myApparentPower SA = sqrt(3) * IA * UA "Apparent power at flowing port A";
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

    model myBreaker "Perfect breaker cutting the current"
      extends Icons.myBreaker;
      extends Icons.myTwoPortsAC;
      Interfaces.myAcausalTerminal terminalA(v(re(start = UNom / sqrt(3)), im(start = 0)), i(re(start = 0), im(start = 0))) "Terminal A of the 2-port node" annotation (
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Interfaces.myAcausalTerminal terminalB(v(re(start = UNom / sqrt(3)), im(start = 0)), i(re(start = 0), im(start = 0))) "Terminal B of the 2-port node" annotation (
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.BooleanInput BrkOpen(start = false) "Breaker start position (false means closed)" annotation (
        Placement(visible = true, transformation(origin = {0, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      parameter Types.myVoltage UNom "Reference voltage of the  breaker";
      Types.myCurrent IA = CM.abs(terminalA.i) "Current flowing at port A";
      Types.myVoltage UA = sqrt(3) * CM.abs(terminalA.v) "Voltage at port A";
      Types.myCurrent IB = CM.abs(terminalB.i) "Current flowing at port B";
      Types.myVoltage UB = sqrt(3) * CM.abs(terminalB.v) "Voltage at port B";
      Types.myApparentPower S = sqrt(3) * IA * UA "Apparent power flowing the breaker";
    equation
      if BrkOpen then
        terminalA.i = Complex(0);
        terminalB.i = Complex(0);
      else
        terminalB.v = terminalA.v;
        terminalA.i + terminalB.i = Complex(0);
      end if;
      annotation (
        Icon(graphics={  Line(points = {{-90, 0}, {-40, 0}}, color = {0, 0, 0}, thickness = 0.5), Line(points = {{90, 0}, {40, 0}}, color = {0, 0, 0}, thickness = 0.5), Line(points = DynamicSelect({{-40, 0}, {20, 40}}, if not BrkOpen then {{-40, 0}, {20, 40}} else {{-40, 0}, {40, 0}}), color = {0, 0, 0}, thickness = 0.5)}),
        Documentation(info = "<html>
    <p>This node prescribes a perfect breaker dropping the current to zero at port B.</p>
    </html>"));
    end myBreaker;

    model mySecondBreaker "Perfect breaker cutting the voltage"
      extends Icons.myBreaker;
      extends Icons.myTwoPortsAC;
      Interfaces.myAcausalTerminal terminalA(v(re(start = UNom / sqrt(3)), im(start = 0)), i(re(start = 0), im(start = 0))) "Terminal A of the 2-port node" annotation (
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Interfaces.myAcausalTerminal terminalB(v(re(start = UNom / sqrt(3)), im(start = 0)), i(re(start = 0), im(start = 0))) "Terminal B of the 2-port node" annotation (
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.BooleanInput BrkOpen(start = false) "Breaker start position is false (closed)" annotation (
        Placement(visible = true, transformation(origin = {0, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      parameter Types.myVoltage UNom "Reference voltage of the  breaker";
      Types.myCurrent IA = CM.abs(terminalA.i) "Current flowing port A";
      Types.myVoltage UA = sqrt(3) * CM.abs(terminalA.v) "Voltage at port A";
      Types.myCurrent IB = CM.abs(terminalB.i) "Current flowing port B";
      Types.myVoltage UB = sqrt(3) * CM.abs(terminalB.v) "Voltage at port B";
      Types.myApparentPower S = sqrt(3) * IA * UA "Apparent power flowing the breaker";
    equation
      if BrkOpen then
        terminalA.i = Complex(0);
        terminalB.v = Complex(0);
      else
        terminalB.v = terminalA.v;
        terminalA.i + terminalB.i = Complex(0);
      end if;
      annotation (
        Icon(graphics={  Line(points = {{-90, 0}, {-40, 0}}, color = {0, 0, 0}, thickness = 0.5), Line(points = {{90, 0}, {40, 0}}, color = {0, 0, 0}, thickness = 0.5), Line(points = DynamicSelect({{-40, 0}, {20, 40}}, if not BrkOpen then {{-40, 0}, {20, 40}} else {{-40, 0}, {40, 0}}), color = {0, 0, 0}, thickness = 0.5)}),
        Documentation(info = "<html>
    <p>This node prescribes a perfect breaker dropping the voltage to zero at port B.</p>
    </html>"));
    end mySecondBreaker;

    model myFailure
    Modelica.Blocks.Interfaces.BooleanInput u annotation (
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.BooleanOutput y annotation (
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      input Integer i = 10;
      parameter Real failureRate = 0.00003 annotation (
        Dialog);
      parameter Real repairRate = 0.001 annotation (
        Dialog);
      Boolean componentIsWorking(start = true, fixed = false);
      Real r "random numbers";
      Real F(start = 0) "cumulative distribution function";
      Real hazardRate "time-varying hazard rates";
    protected
      Integer seed;
    equation
      when {u,componentIsWorking <> pre(componentIsWorking)} then
        // Generate a new random number when initial() or each time a transition occurs
        reinit(F, 0);
        seed = Modelica.Math.Random.Utilities.initializeImpureRandom(Modelica.Math.Random.Utilities.automaticGlobalSeed()+i);
        //We force a different seed for each instance
        r = Modelica.Math.Random.Utilities.impureRandom(id = seed);
      end when;
      hazardRate = if componentIsWorking then failureRate else repairRate;
      der(F) = (1 - F) * hazardRate;
      when F > pre(r) then
        componentIsWorking = not pre(componentIsWorking);
      end when;
      y =  componentIsWorking and u;

    end myFailure;

    package PartialModels "Partial models to be inherited"
      extends Icons.myBasesPackage;

      partial model myPartialSource "Perfect voltage source"
        Interfaces.myAcausalTerminal terminal(v(re(start = UNom / sqrt(3)), im(start = 0))) "Terminal of the 1-port node" annotation (
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        parameter Types.myVoltage UNom "Reference voltage of the source";
        parameter Types.myAngle theta = 0 "Phase shift of voltage phasor";
        Types.myCurrent I = CM.abs(terminal.i) "Current flowing the source";
        Types.myVoltage U = sqrt(3) * CM.abs(terminal.v) "Voltage at the source";
        Types.myApparentPower S = sqrt(3) * I * U "Apparent power flowing the source";
        annotation (
          Icon(coordinateSystem(grid = {0.1, 0.1}, initialScale = 0.1)),
          Documentation(info = "<html>
    <p>This node prescribes the voltage magnitude and phase (default zero) for the source.</p>
</html>"));
      end myPartialSource;

      partial model myPartialLoad "Partial model for loads"
        extends Icons.myLoad;
        extends Icons.myOnePortAC;
        Interfaces.myAcausalTerminal terminal(v(re(start = UNom / sqrt(3)), im(start = 0))) "Terminal of the 1-port node" annotation (
          Placement(visible = true, transformation(origin = {-0.1, -0.3}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-0.1, -0.3}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        parameter Types.myVoltage UNom "Reference voltage of the load";
        parameter Types.myPerUnit lowVoltage(min = 0, max = 1) = if UNom <= 1000 then 0.9 else 0.95 "Lower percentage limit for acceptable voltage";
        parameter Types.myPerUnit highVoltage(min = 1) = if UNom <= 1000 then 1.1 else 1.05 "Higher percentage limit for acceptable voltage";
        parameter Types.myVoltage minU = lowVoltage * UNom "Minimum acceptable value for U";
        parameter Types.myVoltage maxU = highVoltage * UNom "Maximum acceptable value for U";
        parameter Boolean Disengageable = false;
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
        Interfaces.myAcausalTerminal terminalA(v(re(start = UNomA / sqrt(3)), im(start = 0)), i(re(start = 0), im(start = 0))) "Terminal A of the 2-port node" annotation (
          Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Interfaces.myAcausalTerminal terminalB(v(re(start = UNomB / sqrt(3)), im(start = 0)), i(re(start = 0), im(start = 0))) "Terminal B of the 2-port node" annotation (
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
        Types.myCurrent IA = CM.abs(terminalA.i) "Current flowing at port A";
        Types.myVoltage UA = sqrt(3) * CM.abs(terminalA.v) "Voltage at port A";
        Types.myCurrent IB = CM.abs(terminalB.i) "Current flowing at port B";
        Types.myVoltage UB = sqrt(3) * CM.abs(terminalB.v) "Voltage at port B";
        Types.myApparentPower SA = sqrt(3) * IA * UA "Apparent power flowing at port A";
      equation
        if SNom > 0 then
          assert(SA <= 1e3 * SNom, ">>> Apparent power flowing the transformer is exceeding the nominal power for " + getInstanceName(), AssertionLevel.warning);
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
          Icon(coordinateSystem(grid = {0.1, 0.1}), graphics = {Text(extent = {{-102.8, -57.6}, {103.5, -86.4}}, lineColor = {238, 46, 47}, fillColor = {238, 46, 47}, fillPattern = FillPattern.Solid, textString = "fault", textStyle = {TextStyle.Bold})}));
      end myFault;

      partial model mySubNetworkOnePort "Wrapper of sub-network with one port"
        extends Icons.mySubNetwork;
        Interfaces.myAcausalTerminal terminal "Terminal of the 1-port sub-network" annotation (
          Placement(visible = true, transformation(origin = {-98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false), graphics={  Text(origin = {1, 82}, lineColor = {0, 0, 255}, extent = {{-237, 12}, {237, -12}}, textString = "%name")}),
          Diagram(coordinateSystem(preserveAspectRatio = false)));
      end mySubNetworkOnePort;

      partial model mySubNetworkTwoPorts "Wrapper of sub-network with two ports"
        extends Icons.mySubNetwork;
        Interfaces.myAcausalTerminal terminalA "Terminal A of the 2-port sub-network" annotation (
          Placement(visible = true, transformation(origin = {-98, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-98, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Interfaces.myAcausalTerminal terminalB "Terminal B of the 2-port sub-network" annotation (
          Placement(visible = true, transformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Text(origin = {1, 82}, lineColor = {0, 0, 255}, extent = {{-237, 12}, {237, -12}}, textString = "%name")}),
          Diagram(coordinateSystem(preserveAspectRatio = false)));
      end mySubNetworkTwoPorts;
    end PartialModels;
    annotation (
      Documentation(info = "<html><head></head><body>
    <p>The components in this package can be used to build load flow models with a minimum number of equations.</p>
    <p>The goal is to provide an easy way to calculate load flow values for Distribution networks in a Modelica environment.</p>
</body></html>"),
      Icon(graphics={  Text(extent = {{-78, 54}, {72, -58}}, lineColor = {28, 108, 200}, fontName = "Segoe Print", textString = "C")}));
  end Components;

  package Batteries
    extends PowerSysPro.Icons.myPackage;

    package Basics
      extends PowerSysPro.Icons.myBasesPackage;

      model mySignalVoltage "Generic voltage source using the input signal as source voltage"
        extends Modelica.Electrical.Analog.Icons.VoltageSource;
        Modelica.Electrical.Analog.Interfaces.PositivePin p annotation (
          Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
        Modelica.Electrical.Analog.Interfaces.NegativePin n annotation (
          Placement(transformation(extent = {{110, -10}, {90, 10}})));
        Modelica.Blocks.Interfaces.RealInput v(unit = "V") "Voltage between pin p and n (= p.v - n.v) as input signal" annotation (
          Placement(transformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = 270)));
        Batteries.Units.myCurrent i "Current flowing from pin p to pin n";
      equation
        v = p.v - n.v;
        0 = p.i + n.i;
        i = p.i;
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Line(points = {{-109, 20}, {-84, 20}}, color = {160, 160, 164}), Polygon(points = {{-94, 23}, {-84, 20}, {-94, 17}, {-94, 23}}, lineColor = {160, 160, 164}, fillColor = {160, 160, 164}, fillPattern = FillPattern.Solid), Line(points = {{91, 20}, {116, 20}}, color = {160, 160, 164}), Text(extent = {{-109, 25}, {-89, 45}}, textColor = {160, 160, 164}, textString = "i"), Polygon(points = {{106, 23}, {116, 20}, {106, 17}, {106, 23}}, lineColor = {160, 160, 164}, fillColor = {160, 160, 164}, fillPattern = FillPattern.Solid), Text(extent = {{91, 45}, {111, 25}}, textColor = {160, 160, 164}, textString = "i"), Line(points = {{-119, -5}, {-119, 5}}, color = {160, 160, 164}), Line(points = {{-124, 0}, {-114, 0}}, color = {160, 160, 164}), Line(points = {{116, 0}, {126, 0}}, color = {160, 160, 164})}),
          Documentation(revisions = "<html>
<ul>
<li><em> 1998   </em>
       by Martin Otter<br> initially implemented<br>
       </li>
</ul>
</html>", info = "<html>
<p>The signal voltage source is a parameterless converter of real valued signals into a the source voltage. No further effects are modeled. The real valued signal has to be provided by components of the blocks library. It can be regarded as the &quot;Opposite&quot; of a voltage sensor.</p>
</html>"));
      end mySignalVoltage;

      model mySignalCurrent "Generic current source using the input signal as source current"
        extends Modelica.Electrical.Analog.Icons.CurrentSource;
        Modelica.Electrical.Analog.Interfaces.PositivePin p annotation (
          Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
        Modelica.Electrical.Analog.Interfaces.NegativePin n annotation (
          Placement(transformation(extent = {{110, -10}, {90, 10}})));
        Modelica.Blocks.Interfaces.RealInput i(unit = "A") "Current flowing from pin p to pin n as input signal" annotation (
          Placement(transformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = 270)));
        Batteries.Units.myVoltage v "Voltage drop between the two pins (= p.v - n.v)";
      equation
        v = p.v - n.v;
        0 = p.i + n.i;
        i = p.i;
        annotation (
          Documentation(revisions = "<html>
<ul>
<li><em> 1998   </em>
       by Martin Otter<br> initially implemented<br>
       </li>
</ul>
</html>", info = "<html>
<p>The signal current source is a parameterless converter of real valued signals into a the source current. No further effects are modeled. The real valued signal has to be provided by components of the blocks library. It can be regarded as the &quot;Opposite&quot; of a current sensor.</p>
</html>"));
      end mySignalCurrent;

      model myResistor "Ideal electrical resistor"
        parameter Batteries.Units.myResistance R "Resistance";
        extends Modelica.Electrical.Analog.Interfaces.OnePort;
      equation
        v = R * i;
        annotation (
          Documentation(info = "<html>
<p>The linear resistor connects the branch voltage <em>v</em> with the branch current <em>i</em> by <em>i*R = v</em>. The Resistance <em>R</em> is allowed to be positive, zero, or negative.</p>
</html>", revisions = "<html>
<ul>
<li><em> August 07, 2009   </em>
       by Anton Haumer<br> temperature dependency of resistance added<br>
       </li>
<li><em> March 11, 2009   </em>
       by Christoph Clauss<br> conditional heat port added<br>
       </li>
<li><em> 1998   </em>
       by Christoph Clauss<br> initially implemented<br>
       </li>
</ul>
</html>"),Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-70, 30}, {70, -30}}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Line(points = {{-90, 0}, {-70, 0}}, color = {0, 0, 255}), Line(points = {{70, 0}, {90, 0}}, color = {0, 0, 255}), Text(extent = {{-150, 90}, {150, 50}}, textString = "%name", textColor = {0, 0, 255})}));
      end myResistor;

      model myConductor "Ideal electrical conductor"
        parameter Batteries.Units.myConductance G "Conductance";
        extends Modelica.Electrical.Analog.Interfaces.OnePort;
      equation
        i = G * v;
        annotation (
          Documentation(info = "<html>
<p>The linear conductor connects the branch voltage <em>v</em> with the branch current <em>i</em> by <em>i = v*G</em>. The Conductance <em>G</em> is allowed to be positive, zero, or negative.</p>
</html>", revisions = "<html>
<ul>
<li><em> August 07, 2009   </em>
       by Anton Haumer<br> temperature dependency of conductance added<br>
       </li>
<li><em> March 11, 2009   </em>
       by Christoph Clauss<br> conditional heat port added<br>
       </li>
<li><em> 1998   </em>
       by Christoph Clauss<br> initially implemented<br>
       </li>
</ul>
</html>"),Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-70, 30}, {70, -30}}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineColor = {0, 0, 255}), Rectangle(extent = {{-70, 30}, {70, -30}}, lineColor = {0, 0, 255}), Line(points = {{-90, 0}, {-70, 0}}, color = {0, 0, 255}), Line(points = {{70, 0}, {90, 0}}, color = {0, 0, 255}), Text(extent = {{-150, 90}, {150, 50}}, textString = "%name", textColor = {0, 0, 255})}));
      end myConductor;

      model myGround "Ground node"
        Modelica.Electrical.Analog.Interfaces.Pin p annotation (
          Placement(transformation(origin = {0, 100}, extent = {{10, -10}, {-10, 10}}, rotation = 270)));
      equation
        p.v = 0;
        annotation (
          Documentation(info = "<html>
<p>Ground of an electrical circuit. The potential at the ground node is zero. Every electrical circuit has to contain at least one ground object.</p>
</html>", revisions = "<html>
<ul>
<li><em> 1998   </em>
       by Christoph Clauss<br> initially implemented<br>
       </li>
</ul>
</html>"),Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Line(points = {{-60, 50}, {60, 50}}, color = {0, 0, 255}), Line(points = {{-40, 30}, {40, 30}}, color = {0, 0, 255}), Line(points = {{-20, 10}, {20, 10}}, color = {0, 0, 255}), Line(points = {{0, 90}, {0, 50}}, color = {0, 0, 255}), Text(extent = {{-150, -10}, {150, -50}}, textString = "%name", textColor = {0, 0, 255})}));
      end myGround;

      block myIntegrator "simplified current integrator"
        parameter Real k "Integrator gain";
        parameter Real outMax "Upper limit of output";
        parameter Real outMin "Lower limit of output";
        Modelica.Blocks.Interfaces.RealOutput q "Charge as current integration" annotation (
          Placement(transformation(extent = {{100, -10}, {120, 10}}), iconTransformation(extent = {{100, -10}, {120, 10}})));
        Modelica.Blocks.Interfaces.RealInput i "Current as input signal" annotation (
          Placement(transformation(extent = {{-126, -14}, {-100, 12}}), iconTransformation(extent = {{-100, -12}, {-76, 12}})));
      equation
        der(q) = if q < outMin and k * i < 0 or q > outMax and k * i > 0 then 0 else k * i;
        annotation (
          Documentation(info = "<html>
<p>
This blocks computes <strong>y</strong> as <em>integral</em>
of the input <strong>u</strong> multiplied with the gain <em>k</em>. If the
integral reaches a given upper or lower <em>limit</em> and the
input will drive the integral outside of this bound, the
integration is halted and only restarted if the input drives
the integral away from the bounds.
</p>

<p>
It might be difficult to initialize the integrator in steady state.
This is discussed in the description of package
<a href=\"modelica://Modelica.Blocks.Continuous#info\">Continuous</a>.
</p>

<p>
If parameter <strong>limitsAtInit</strong> = <strong>false</strong>, the limits of the
integrator are removed from the initialization problem which
leads to a much simpler equation system. After initialization has been
performed, it is checked via an assert whether the output is in the
defined limits. For backward compatibility reasons
<strong>limitsAtInit</strong> = <strong>true</strong>. In most cases it is best
to use <strong>limitsAtInit</strong> = <strong>false</strong>.
</p>
<p>
If the <em>reset</em> port is enabled, then the output <strong>y</strong> is reset to <em>set</em>
or to <em>y_start</em> (if the <em>set</em> port is not enabled), whenever the <em>reset</em>
port has a rising edge.
</p>
</html>"),Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Line(points = {{-80, 78}, {-80, -90}}, color = {192, 192, 192}), Polygon(points = {{-80, 90}, {-88, 68}, {-72, 68}, {-80, 90}}, lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid), Line(points = {{-90, -80}, {82, -80}}, color = {192, 192, 192}), Polygon(points = {{90, -80}, {68, -72}, {68, -88}, {90, -80}}, lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid), Line(points = DynamicSelect({{-80, -80}, {20, 20}, {80, 20}}, if use_reset then {{-80, -80}, {20, 20}, {60, 20}, {60, -80}, {80, -60}} else {{-80, -80}, {20, 20}, {80, 20}}), color = {0, 0, 127}), Line(visible = strict, points = DynamicSelect({{20, 20}, {80, 20}}, if use_reset then {{20, 20}, {60, 20}} else {{20, 20}, {80, 20}}), color = {255, 0, 0}), Rectangle(extent = {{-100, -100}, {100, 100}}, lineColor = {0, 0, 127}), Text(extent = {{-152, 20}, {-112, -18}}, textColor = {0, 0, 127}, textString = "i")}));
      end myIntegrator;

      block myCombiTable "Table look-up in with one input and one output"
        extends Modelica.Blocks.Interfaces.SISO;
        parameter Real table[:, :] = fill(0.0, 0, 2) "Table matrix";
      protected
        parameter Modelica.Blocks.Types.ExternalCombiTable1D tableID = Modelica.Blocks.Types.ExternalCombiTable1D("NoName", "NoName", table, 2:2, Modelica.Blocks.Types.Smoothness.LinearSegments, Modelica.Blocks.Types.Extrapolation.LastTwoPoints, false);
      equation
        assert(size(table, 1) > 0 and size(table, 2) > 0, "table is not structurally correct");
        y = Modelica.Blocks.Tables.Internal.getTable1DValue(tableID, 1, u);
        annotation (
          Documentation(info = "<html>
<p>
<strong>Univariate constant</strong>, <strong>linear</strong> or <strong>cubic Hermite
spline interpolation</strong> in <strong>one</strong> dimension of a
<strong>table</strong>.
Via parameter <strong>columns</strong> it can be defined how many columns of the
table are interpolated. If, e.g., columns={2,4}, it is assumed that
2 output signals are present and that the first output interpolates
via column 2 and the second output interpolates via column 4 of the
table matrix.
</p>
<p>
The grid points and function values are stored in a matrix \"table[i,j]\",
where the first column \"table[:,1]\" contains the grid points and the
other columns contain the data to be interpolated. Example:
</p>
<blockquote><pre>
table = [0,  0;
         1,  1;
         2,  4;
         4, 16]
If, e.g., the input u = 1.0, the output y =  1.0,
    e.g., the input u = 1.5, the output y =  2.5,
    e.g., the input u = 2.0, the output y =  4.0,
    e.g., the input u =-1.0, the output y = -1.0 (i.e., extrapolation).
</pre></blockquote>
<ul>
<li>The interpolation interval is found by a binary search where the interval used in the
    last call is used as start interval.</li>
<li>Via parameter <strong>smoothness</strong> it is defined how the data is interpolated:
<blockquote><pre>
smoothness = 1: Linear interpolation
           = 2: Akima interpolation: Smooth interpolation by cubic Hermite
                splines such that der(y) is continuous, also if extrapolated.
           = 3: Constant segments
           = 4: Fritsch-Butland interpolation: Smooth interpolation by cubic
                Hermite splines such that y preserves the monotonicity and
                der(y) is continuous, also if extrapolated.
           = 5: Steffen interpolation: Smooth interpolation by cubic Hermite
                splines such that y preserves the monotonicity and der(y)
                is continuous, also if extrapolated.
           = 6: Modified Akima interpolation: Smooth interpolation by cubic
                Hermite splines such that der(y) is continuous, also if
                extrapolated. Additionally, overshoots and edge cases of the
                original Akima interpolation method are avoided.
</pre></blockquote></li>
<li>First and second <strong>derivatives</strong> are provided, with exception of the following two smoothness options.
<ol>
<li>No derivatives are provided for interpolation by constant segments.</li>
<li>No second derivative is provided for linear interpolation.</li>
</ol></li>
<li>Values <strong>outside</strong> of the table range, are computed by
    extrapolation according to the setting of parameter <strong>extrapolation</strong>:
<blockquote><pre>
extrapolation = 1: Hold the first or last value of the table,
                   if outside of the table scope.
              = 2: Extrapolate by using the derivative at the first/last table
                   points if outside of the table scope.
                   (If smoothness is LinearSegments or ConstantSegments
                   this means to extrapolate linearly through the first/last
                   two table points.).
              = 3: Periodically repeat the table data (periodical function).
              = 4: No extrapolation, i.e. extrapolation triggers an error
</pre></blockquote></li>
<li>If the table has only <strong>one row</strong>, the table value is returned,
    independent of the value of the input signal.</li>
<li>The grid values (first column) have to be strictly increasing.</li>
</ul>
<p>
The table matrix can be defined in the following ways:
</p>
<ol>
<li>Explicitly supplied as <strong>parameter matrix</strong> \"table\",
    and the other parameters have the following values:
<blockquote><pre>
tableName is \"NoName\" or has only blanks,
fileName  is \"NoName\" or has only blanks.
</pre></blockquote></li>
<li><strong>Read</strong> from a <strong>file</strong> \"fileName\" where the matrix is stored as
    \"tableName\". Both text and MATLAB MAT-file format is possible.
    (The text format is described below).
    The MAT-file format comes in four different versions: v4, v6, v7 and v7.3.
    The library supports at least v4, v6 and v7 whereas v7.3 is optional.
    It is most convenient to generate the MAT-file from FreeMat or MATLAB&reg;
    by command
<blockquote><pre>
save tables.mat tab1 tab2 tab3
</pre></blockquote>
    or Scilab by command
<blockquote><pre>
savematfile tables.mat tab1 tab2 tab3
</pre></blockquote>
    when the three tables tab1, tab2, tab3 should be used from the model.<br>
    Note, a fileName can be defined as URI by using the helper function
    <a href=\"modelica://Modelica.Utilities.Files.loadResource\">loadResource</a>.</li>
<li>Statically stored in function \"usertab\" in file \"usertab.c\".
    The matrix is identified by \"tableName\". Parameter
    fileName = \"NoName\" or has only blanks. Row-wise storage is always to be
    preferred as otherwise the table is reallocated and transposed.
    See the <a href=\"modelica://Modelica.Blocks.Tables\">Tables</a> package
    documentation for more details.</li>
</ol>
<p>
When the constant \"NO_FILE_SYSTEM\" is defined, all file I/O related parts of the
source code are removed by the C-preprocessor, such that no access to files takes place.
</p>
<p>
If tables are read from a text file, the file needs to have the
following structure (\"-----\" is not part of the file content):
</p>
<blockquote><pre>
-----------------------------------------------------
#1
double tab1(5,2)   # comment line
  0   0
  1   1
  2   4
  3   9
  4  16
double tab2(5,2)   # another comment line
  0   0
  2   2
  4   8
  6  18
  8  32
-----------------------------------------------------
</pre></blockquote>
<p>
Note, that the first two characters in the file need to be
\"#1\" (a line comment defining the version number of the file format).
Afterwards, the corresponding matrix has to be declared
with type (= \"double\" or \"float\"), name and actual dimensions.
Finally, in successive rows of the file, the elements of the matrix
have to be given. The elements have to be provided as a sequence of
numbers in row-wise order (therefore a matrix row can span several
lines in the file and need not start at the beginning of a line).
Numbers have to be given according to C syntax (such as 2.3, -2, +2.e4).
Number separators are spaces, tab (\\t), comma (,), or semicolon (;).
Several matrices may be defined one after another. Line comments start
with the hash symbol (#) and can appear everywhere.
Text files should either be ASCII or UTF-8 encoded, where UTF-8 encoded strings are only allowed in line comments and an optional UTF-8 BOM at the start of the text file is ignored.
Other characters, like trailing non comments, are not allowed in the file.
</p>
<p>
MATLAB is a registered trademark of The MathWorks, Inc.
</p>
</html>"),Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100.0, -100.0}, {100.0, 100.0}}), graphics = {Line(points = {{-60.0, 40.0}, {-60.0, -40.0}, {60.0, -40.0}, {60.0, 40.0}, {30.0, 40.0}, {30.0, -40.0}, {-30.0, -40.0}, {-30.0, 40.0}, {-60.0, 40.0}, {-60.0, 20.0}, {60.0, 20.0}, {60.0, 0.0}, {-60.0, 0.0}, {-60.0, -20.0}, {60.0, -20.0}, {60.0, -40.0}, {-60.0, -40.0}, {-60.0, 40.0}, {60.0, 40.0}, {60.0, -40.0}}), Line(points = {{0.0, 40.0}, {0.0, -40.0}}), Rectangle(fillColor = {255, 215, 136}, fillPattern = FillPattern.Solid, extent = {{-60.0, 20.0}, {-30.0, 40.0}}), Rectangle(fillColor = {255, 215, 136}, fillPattern = FillPattern.Solid, extent = {{-60.0, 0.0}, {-30.0, 20.0}}), Rectangle(fillColor = {255, 215, 136}, fillPattern = FillPattern.Solid, extent = {{-60.0, -20.0}, {-30.0, 0.0}}), Rectangle(fillColor = {255, 215, 136}, fillPattern = FillPattern.Solid, extent = {{-60.0, -40.0}, {-30.0, -20.0}})}));
      end myCombiTable;

      block myGain "Output the product of a gain value with the input signal"
        parameter Real k(start = 1, unit = "1") "Gain value multiplied with input signal";
        Modelica.Blocks.Interfaces.RealInput u "Input signal connector" annotation (
          Placement(transformation(extent = {{-140, -20}, {-100, 20}})));
        Modelica.Blocks.Interfaces.RealOutput y "Output signal connector" annotation (
          Placement(transformation(extent = {{100, -10}, {120, 10}})));
      equation
        y = k * u;
        annotation (
          Documentation(info = "<html>
<p>
This block computes output <em>y</em> as
<em>product</em> of gain <em>k</em> with the
input <em>u</em>:
</p>
<blockquote><pre>
y = k * u;
</pre></blockquote>

</html>"),Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Polygon(points = {{-100, -100}, {-100, 100}, {100, 0}, {-100, -100}}, lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-150, -140}, {150, -100}}, textString = "k=%k"), Text(extent = {{-150, 140}, {150, 100}}, textString = "%name", textColor = {0, 0, 255})}));
      end myGain;

      model myInverter "Perfect inverter"
        Modelica.Electrical.Analog.Interfaces.PositivePin p "Positive DC input" annotation (
          Placement(transformation(extent = {{-110, 70}, {-90, 50}})));
        Modelica.Electrical.Analog.Interfaces.NegativePin n "Negative DC input" annotation (
          Placement(transformation(extent = {{-110, -70}, {-90, -50}})));
        Modelica.Blocks.Interfaces.RealInput i(unit = "A") "Current flowing from pin p to pin n as input signal" annotation (
          Placement(transformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = 270)));
        parameter PowerSysPro.Batteries.Units.myVoltage V_out;
        //PowerSysPro.Batteries.Units.myCurrent I_out;
        input Boolean ChargingMode "true means charging mode";
      equation
        0 = p.i + n.i;
        if ChargingMode then
          i = p.i;
        else
          1000 * V_out = -(p.v - n.v);
        end if;
      end myInverter;
    end Basics;

    package Components
      extends PowerSysPro.Icons.myPackage;

      model myIdealBattery "Battery with OCV as a function of SOC, self-discharge and inner resistance"
        // Get the battery parameters from dataset
        extends Batteries.Icons.myBattery;
        replaceable parameter DataSet.myDataSetNb1 dataSet "Set of parameters" annotation (
          Dialog(group = "Battery parameters"));
        parameter Integer Ns(min = 1) "Number of serial connected cells" annotation (
          Dialog(group = "Battery parameters"));
        parameter Integer Np(min = 1) "Number of parallel connected cells" annotation (
          Dialog(group = "Battery parameters"));
        parameter Types.myPerUnit SOC_start "Initial SOC" annotation (
          Dialog(group = "OCV and SOC parameters"));
        // Define the OCV(SOC) function
        Batteries.Basics.myCombiTable ocv_soc(table = OCV_SOC) annotation (
          Placement(transformation(extent = {{-4, 66}, {16, 86}})));
        Batteries.Basics.myIntegrator integrator(k = 1 / (Np * 3600 * dataSet.QNom), outMax = 1 - SOCtolerance, outMin = SOCtolerance) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 0, origin = {-24, 76})));
        Basics.myGain gainV(k = Ns * dataSet.OCVmax) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {22, 46})));
        Types.myPerUnit SOC(start = SOC_start) = integrator.q "State of charge";
        parameter Real maxCharging = 0.8 "Maximum SOC in charging mode" annotation (
          Dialog(group = "Charging/Discharging parameters"));
        parameter Real minCharging = 0.2 "Minimum SOC in discharging mode" annotation (
          Dialog(group = "Charging/Discharging parameters"));
        // Define the OCV circuit
        extends Batteries.Interfaces.myTwoPin;
        Basics.myResistor r0(R = Ns * dataSet.R0 / Np) annotation (
          Placement(transformation(extent = {{52, -10}, {72, 10}})));
        Basics.myConductor selfDis(G = Np * dataSet.Idis / (Ns * dataSet.OCVmax)) annotation (
          Placement(transformation(extent = {{12, -34}, {32, -14}})));
        Basics.mySignalVoltage ocv annotation (
          Placement(transformation(extent = {{12, -10}, {32, 10}})));
      protected
        parameter Types.myPerUnit SOCtolerance = 1e-9 "Tolerance to detect depleted of overcharged battery";
        parameter Types.myPerUnit OCV_SOC[:, 2] = dataSet.OCV_SOC;
      equation
        assert(dataSet.OCVmax > dataSet.OCVmin, "Specify 0 <= OCVmin < OCVmax");
        assert(dataSet.SOCmax > dataSet.SOCmin, "Specify 0 <= SOCmin < SOCmax <= 1");
        assert(OCV_SOC[1, 1] >= 0, "Specify OCV(SOC) table with minimum SOC >= 0");
        assert(OCV_SOC[end, 1] <= 1, "Specify OCV(SOC) table with maximum SOC <= 1");
        assert(OCV_SOC[1, 2] >= 0, "Specify OCV(SOC) table with minimum OCV/OCVmax >= 0");
        assert(OCV_SOC[end, 2] <= 1, "Specify OCV(SOC) table with maximum OCV/OCVmax <= 1");
        integrator.i = p.i;
        connect(gainV.y, ocv.v) annotation (
          Line(points = {{22, 35}, {22, 12}}, color = {0, 0, 127}));
        connect(ocv_soc.y, gainV.u) annotation (
          Line(points = {{17, 76}, {22, 76}, {22, 58}}, color = {0, 0, 127}));
        connect(ocv.n, r0.p) annotation (
          Line(points = {{32, 0}, {52, 0}}, color = {0, 0, 255}));
        connect(r0.n, n) annotation (
          Line(points = {{72, 0}, {100, 0}}, color = {0, 0, 255}));
        connect(p, ocv.p) annotation (
          Line(points = {{-100, 0}, {12, 0}}, color = {0, 0, 255}));
        connect(ocv.n, selfDis.n) annotation (
          Line(points = {{32, 0}, {40, 0}, {40, -24}, {32, -24}}, color = {0, 0, 255}));
        connect(selfDis.p, ocv.p) annotation (
          Line(points = {{12, -24}, {4, -24}, {4, 0}, {12, 0}}, color = {0, 0, 255}));
        connect(ocv_soc.u, integrator.q) annotation (
          Line(points = {{-6, 76}, {-13, 76}}, color = {0, 0, 127}));
        annotation (
          Documentation(info = "<html>
<p>
The battery is modeled by open-circuit voltage (OCV) dependent on state of charge (SOC), a self-discharge component and an inner resistance.<br>
Parameters are collected in parameter record <a href=\"modelica://Modelica.Electrical.Batteries.ParameterRecords.CellData\">cellData</a>.<br>
All losses are dissipated to the optional <code>heatPort</code>.
</p>
<p>
For details, see <a href=\"modelica://Modelica.Electrical.Batteries.UsersGuide.Concept\">concept</a> and <a href=\"modelica://Modelica.Electrical.Batteries.UsersGuide.Parameterization\">parameterization</a>.
</p>
<h4>Note</h4>
<p>
SOC &gt; SOCmax and SOC &lt; SOCmin triggers an error.
</p>
</html>"),Diagram(graphics = {Text(origin = {14, -10}, lineColor = {28, 108, 200}, extent = {{-134, 72}, {-18, 12}}, textString = "The current part of the positive pin p
is given as an entry of the integrator")}));
      end myIdealBattery;
      annotation (
        Icon(graphics = {Text(extent = {{-74, 60}, {76, -52}}, lineColor = {28, 108, 200}, fontName = "Segoe Print", textString = "C")}));
    end Components;

    package Interfaces
      extends PowerSysPro.Icons.myInterfacesPackage;

      partial model myTwoPin "Component with two electrical pins"
        Batteries.Interfaces.myPositivePin p "Positive electrical pin" annotation (
          Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
        Batteries.Interfaces.myNegativePin n "Negative electrical pin" annotation (
          Placement(transformation(extent = {{90, -10}, {110, 10}})));
        annotation (
          Documentation(revisions = "<html>
<ul>
<li><em> 1998   </em>
       by Christoph Clauss<br> initially implemented<br>
       </li>
</ul>
</html>", info = "<html>
<p>TwoPin is a partial model with two pins and one internal variable for the voltage over the two pins. Internal currents are not defined. It is intended to be used in cases where the model which inherits TwoPin is composed by combining other components graphically, not by equations.</p>
</html>"));
      end myTwoPin;

      connector myPositivePin "Positive pin of an electrical component"
        Batteries.Units.myElectricPotential v "Potential at the pin" annotation (
          unassignedMessage = "An electrical potential cannot be uniquely calculated.
The reason could be that
- a ground object is missing (Modelica.Electrical.Analog.Basic.Ground)
  to define the zero potential of the electrical circuit, or
- a connector of an electrical component is not connected.");
        flow Batteries.Units.myCurrent i "Current flowing into the pin" annotation (
          unassignedMessage = "An electrical current cannot be uniquely calculated.
The reason could be that
- a ground object is missing (Modelica.Electrical.Analog.Basic.Ground)
  to define the zero potential of the electrical circuit, or
- a connector of an electrical component is not connected.");
        annotation (
          defaultComponentName = "pin_p",
          Documentation(info = "<html>
<p>Connectors PositivePin and NegativePin are nearly identical. The only difference is that the icons are different in order to identify more easily the pins of a component. Usually, connector PositivePin is used for the positive and connector NegativePin for the negative pin of an electrical component.</p>
</html>", revisions = "<html>
<ul>
<li><em> 1998   </em>
       by Christoph Clauss<br> initially implemented<br>
       </li>
</ul>
</html>"),Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid)}),
          Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Rectangle(extent = {{-40, 40}, {40, -40}}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-160, 110}, {40, 50}}, textColor = {0, 0, 255}, textString = "%name")}));
      end myPositivePin;

      connector myNegativePin "Negative pin of an electrical component"
        Batteries.Units.myElectricPotential v "Potential at the pin" annotation (
          unassignedMessage = "An electrical potential cannot be uniquely calculated.
The reason could be that
- a ground object is missing (Modelica.Electrical.Analog.Basic.Ground)
  to define the zero potential of the electrical circuit, or
- a connector of an electrical component is not connected.");
        flow Batteries.Units.myCurrent i "Current flowing into the pin" annotation (
          unassignedMessage = "An electrical current cannot be uniquely calculated.
The reason could be that
- a ground object is missing (Modelica.Electrical.Analog.Basic.Ground)
  to define the zero potential of the electrical circuit, or
- a connector of an electrical component is not connected.");
        annotation (
          defaultComponentName = "pin_n",
          Documentation(info = "<html>
<p>Connectors PositivePin and NegativePin are nearly identical. The only difference is that the icons are different in order to identify more easily the pins of a component. Usually, connector PositivePin is used for the positive and connector NegativePin for the negative pin of an electrical component.</p>
</html>", revisions = "<html>
<dl>
<dt><em>1998</em></dt>
<dd>by Christoph Clauss initially implemented
</dd>
</dl>
</html>"),Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid)}),
          Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics={  Rectangle(extent = {{-40, 40}, {40, -40}}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-40, 110}, {160, 50}}, textString = "%name", textColor = {0, 0, 255})}));
      end myNegativePin;
    end Interfaces;

    package DataSet
      extends PowerSysPro.Icons.myPackage;

      record myDataSetNb0 "Set #0 of cell parameters"
        extends Batteries.Icons.myDataSet;
        parameter Batteries.Units.myElectricCharge QNom = 5 "Nominal (maximum) cell charge" annotation (
          Dialog(group = "Cell parameters"));
        parameter Batteries.Types.myPerUnit IchCoeff = 0.1 "Maximum charging current is IchCoeff*QNom" annotation (
          Dialog(group = "Cell parameters"));
        parameter Batteries.Units.myCurrent Idis = 0 "Self-discharge current at SOC = SOCmax" annotation (
          Dialog(group = "Battery parameters"));
        parameter Batteries.Units.myResistance R0 = OCVmax / 1200 "Total inner resistance ! Isc=1200??" annotation (
          Dialog(group = "Battery parameters"));
        parameter Batteries.Types.myPerUnit SOCmax = 1 "Maximum state of charge" annotation (
          Dialog(group = "OCV and SOC parameters"));
        parameter Batteries.Types.myPerUnit SOCmin = 0 "Minimum state of charge" annotation (
          Dialog(group = "OCV and SOC parameters"));
        parameter Batteries.Units.myVoltage OCVmax = 4.2 "OCV at SOC = SOCmax" annotation (
          Dialog(group = "OCV and SOC parameters"));
        parameter Batteries.Units.myVoltage OCVmin = 2.5 "OCV at SOC = SOCmin" annotation (
          Dialog(group = "OCV and SOC parameters"));
        parameter Real OCV_SOC[:, 2] = [SOCmin, OCVmin / OCVmax; SOCmax, 1] "OCV(SOC) table" annotation (
          Dialog(group = "OCV and SOC parameters"));
        annotation (
          defaultComponentPrefixes = "parameter",
          Documentation(info = "<html>
  <p>This is an example for an OCV versus SOC characteristic</p>
</html>"),Icon(graphics = {Text(textColor = {0, 0, 255}, extent = {{-114, 6}, {106, 36}}, textString = "Ns = %Ns"), Text(textColor = {0, 0, 255}, extent = {{-136, -40}, {118, -12}}, textString = "Np = %Np"), Text(textColor = {0, 0, 255}, extent = {{-176, -88}, {162, -58}}, textString = "QNom = %QNom")}));
      end myDataSetNb0;

      record myDataSetNb1 "Set #1 of cell parameters"
        extends Batteries.Icons.myDataSet;
        parameter Batteries.Units.myElectricCharge QNom = 5 "Nominal (maximum) cell charge" annotation (
          Dialog(group = "Cell parameters"));
        parameter Batteries.Types.myPerUnit IchCoeff = 0.1 "Maximum charging current is IchCoeff*QNom" annotation (
          Dialog(group = "Cell parameters"));
        parameter Batteries.Units.myCurrent Idis = 0 "Self-discharge current at SOC = SOCmax" annotation (
          Dialog(group = "Battery parameters"));
        parameter Batteries.Units.myResistance R0 = OCVmax / 1200 "Total inner resistance ! Isc=1200??" annotation (
          Dialog(group = "Battery parameters"));
        parameter Batteries.Types.myPerUnit SOCmax = 1 "Maximum state of charge" annotation (
          Dialog(group = "OCV and SOC parameters"));
        parameter Batteries.Types.myPerUnit SOCmin = 0 "Minimum state of charge" annotation (
          Dialog(group = "OCV and SOC parameters"));
        parameter Batteries.Units.myVoltage OCVmax = 4.2 "OCV at SOC = SOCmax" annotation (
          Dialog(group = "OCV and SOC parameters"));
        parameter Batteries.Units.myVoltage OCVmin = 2.5 "OCV at SOC = SOCmin" annotation (
          Dialog(group = "OCV and SOC parameters"));
        parameter Real OCV_SOC[:, 2] = [0.00, 0.595; 0.01, 0.656; 0.02, 0.688; 0.03, 0.714; 0.04, 0.737; 0.05, 0.744; 0.06, 0.750; 0.07, 0.754; 0.08, 0.756; 0.09, 0.758; 0.10, 0.761; 0.15, 0.774; 0.20, 0.786; 0.25, 0.795; 0.30, 0.804; 0.35, 0.811; 0.40, 0.818; 0.45, 0.826; 0.50, 0.835; 0.55, 0.843; 0.60, 0.855; 0.65, 0.871; 0.70, 0.888; 0.75, 0.905; 0.80, 0.926; 0.85, 0.943; 0.90, 0.964; 0.95, 0.980; 1.00, 1.00] "OCV(SOC) table" annotation (
          Dialog(group = "OCV and SOC parameters"));
        annotation (
          defaultComponentPrefixes = "parameter",
          Documentation(info = "<html>
  <p>This is an example for an OCV versus SOC characteristic</p>
</html>"),Icon(graphics = {Text(textColor = {0, 0, 255}, extent = {{-114, 6}, {106, 36}}, textString = "Ns = %Ns"), Text(textColor = {0, 0, 255}, extent = {{-136, -40}, {118, -12}}, textString = "Np = %Np"), Text(textColor = {0, 0, 255}, extent = {{-176, -88}, {162, -58}}, textString = "QNom = %QNom")}));
      end myDataSetNb1;
      annotation (
        Icon(graphics = {Text(extent = {{-70, 60}, {80, -52}}, lineColor = {28, 108, 200}, fontName = "Segoe Print", textString = "D")}));
    end DataSet;

    package Examples
      extends PowerSysPro.Icons.myExamplesPackage;

      model BatteryDischargeCharge "Discharge and charge idealized battery"
        extends Modelica.Icons.Example;
        // dataset myDataSetNb0 is used to compare results with BatteryDischargeChargeFromMSL model
        Components.myIdealBattery battery(redeclare Batteries.DataSet.myDataSetNb0 dataSet, Ns = 10, Np = 2, SOC_start = 0.95) annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 270, origin = {40, 0})));
        Basics.myGround ground annotation (
          Placement(transformation(extent = {{10, -40}, {30, -20}})));
        Basics.mySignalCurrent current annotation (
          Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 90)));
        Utilities.PulseSeries pulseSeries(n1 = 7, T1 = 60, Tp1 = 60, n2 = 7, Tp = 60, k = 75, startTime = 60) annotation (
          Placement(transformation(extent = {{-54, -10}, {-34, 10}})));
      equation
        connect(battery.n, ground.p) annotation (
          Line(points = {{40, -10}, {40, -20}, {20, -20}}, color = {0, 0, 255}));
        connect(current.n, ground.p) annotation (
          Line(points = {{0, -10}, {0, -20}, {20, -20}}, color = {0, 0, 255}));
        connect(pulseSeries.y, current.i) annotation (
          Line(points = {{-33, 0}, {-12, 0}}, color = {0, 0, 127}));
        connect(current.p, battery.p) annotation (
          Line(points = {{0, 10}, {0, 24}, {40, 24}, {40, 10}}, color = {0, 0, 255}));
        annotation (
          experiment(StopTime = 3600, Interval = 0.1, Tolerance = 1e-06, __Dymola_Algorithm = "Dassl"),
          Documentation(info = "<html>
<p>Two batteries with a nominal charge of 10 A.h starting with an initial SOC = 95 % are compared:</p>
<ul>
<li><code>battery1</code> is a battery OCV linearly dependent on SOC, without self-discharge and not comprising RC-elements.</li>
<li><code>battery2</code> is a battery OCV dependency on SOC is specified by a table, with self-discharge and including RC-elements.</li>
</ul>
<p>
Two <a href=\"modelica://Modelica.Electrical.Batteries.ParameterRecords.CellData\">parameter records</a> <code>cellData1</code> and <code>cellData2</code> are used to parameterize the battery models.
</p>
<p>
First the batteries are discharged with 7 current pulses of 50 A for 1 minute, and breaks between the pulses of 1 minute, ending at SOC = 5 %.<br>
Subsequently, the batteries are charged again with 7 current pulses of 50 A for 1 minute, and breaks between the pulses of 1 minute, ending at SOC = 95 % again.<br>
In the end, the batteries are in no-load condition to reveal self-discharge effects.
Note that self-discharge of <code>battery2</code> is set to an unrealistic high value, to show self-discharge within a rather short time span.<br>
The parameters of the RC-elements of <code>battery2</code> are set to estimated values, just to demonstrate the effects.
</p>
<p>Simulate and plot terminal voltage <code>battery1.v</code> and <code>battery2.v</code> as well as state of charge <code>battery1.SOC</code> and <code>battery2.SOC</code>.</p>
<p>
Plotting <code>energy1.y</code> and <code>energy2.y</code>, it is remarkable that first energy is delivered by the battery,
but then due to the losses more energy is consumed to recharge the battery.
</p>
</html>"),Diagram(graphics = {Text(extent = {{-108, -30}, {114, -90}}, textColor = {0, 0, 127}, textString = "same as Modelica.Electrical.Batteries.Examples.BatteryDischargeCharge
with CellStack battery")}));
      end BatteryDischargeCharge;

      model BatteryDischargeChargeFromMSL "Discharge and charge idealized battery"
        extends Modelica.Icons.Example;
        // This model is derived from BatteryDischargeCharge in Modelica.Electrical.Batteries.Examples
        parameter Modelica.Electrical.Batteries.ParameterRecords.CellData cellData(Qnom = 18000, OCVmax = 4.2, OCVmin = 2.5, Ri = cellData.OCVmax / 1200) annotation (
          Placement(transformation(extent = {{62, -18}, {82, 2}})));
        Modelica.Electrical.Batteries.BatteryStacks.CellStack battery(Ns = 10, Np = 2, cellData = cellData, useHeatPort = false, SOC(fixed = true, start = 0.95)) annotation (
          Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 270, origin = {42, 2})));
        Basics.myGround ground annotation (
          Placement(transformation(extent = {{12, -38}, {32, -18}})));
        Basics.mySignalCurrent signalCurrent1 annotation (
          Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 90, origin = {2, 2})));
        Utilities.PulseSeries pulseSeries(n1 = 7, T1 = 60, Tp1 = 60, n2 = 7, Tp = 60, k = 75, startTime = 60) annotation (
          Placement(transformation(extent = {{-56, -8}, {-36, 12}})));
      equation
        connect(battery.n, ground.p) annotation (
          Line(points = {{42, -8}, {42, -18}, {22, -18}}, color = {0, 0, 255}));
        connect(signalCurrent1.n, ground.p) annotation (
          Line(points = {{2, -8}, {2, -18}, {22, -18}}, color = {0, 0, 255}));
        connect(signalCurrent1.p, battery.p) annotation (
          Line(points = {{2, 12}, {2, 32}, {42, 32}, {42, 12}}, color = {0, 0, 255}));
        connect(signalCurrent1.i, pulseSeries.y) annotation (
          Line(points = {{-10, 2}, {-35, 2}}, color = {0, 0, 127}));
        annotation (
          experiment(StopTime = 3600, Interval = 0.1, Tolerance = 1e-06),
          Documentation(info = "<html>
<p>Two batteries with a nominal charge of 10 A.h starting with an initial SOC = 95 % are compared:</p>
<ul>
<li><code>battery1</code> is a battery OCV linearly dependent on SOC, without self-discharge and not comprising RC-elements.</li>
<li><code>battery2</code> is a battery OCV dependency on SOC is specified by a table, with self-discharge and including RC-elements.</li>
</ul>
<p>
Two <a href=\"modelica://Modelica.Electrical.Batteries.ParameterRecords.CellData\">parameter records</a> <code>cellData1</code> and <code>cellData2</code> are used to parameterize the battery models.
</p>
<p>
First the batteries are discharged with 7 current pulses of 50 A for 1 minute, and breaks between the pulses of 1 minute, ending at SOC = 5 %.<br>
Subsequently, the batteries are charged again with 7 current pulses of 50 A for 1 minute, and breaks between the pulses of 1 minute, ending at SOC = 95 % again.<br>
In the end, the batteries are in no-load condition to reveal self-discharge effects.
Note that self-discharge of <code>battery2</code> is set to an unrealistic high value, to show self-discharge within a rather short time span.<br>
The parameters of the RC-elements of <code>battery2</code> are set to estimated values, just to demonstrate the effects.
</p>
<p>Simulate and plot terminal voltage <code>battery1.v</code> and <code>battery2.v</code> as well as state of charge <code>battery1.SOC</code> and <code>battery2.SOC</code>.</p>
<p>
Plotting <code>energy1.y</code> and <code>energy2.y</code>, it is remarkable that first energy is delivered by the battery,
but then due to the losses more energy is consumed to recharge the battery.
</p>
</html>"),Diagram(graphics = {Text(extent = {{-108, -30}, {114, -90}}, textColor = {0, 0, 127}, textString = "same as Modelica.Electrical.Batteries.Examples.BatteryDischargeCharge
with CellStack battery")}));
      end BatteryDischargeChargeFromMSL;
    end Examples;

    package Units
      extends PowerSysPro.Icons.myPackage;
      type myVoltage = Modelica.Units.SI.ElectricPotential;
      type myCurrent = Modelica.Units.SI.ElectricCurrent;
      type myElectricCharge = Real(final quantity = "ElectricCharge", displayUnit = "A.h", unit = "A.h");
      type myResistance = Real(final quantity = "Resistance", final unit = "Ohm");
      type myConductance = Real(final quantity = "Conductance", final unit = "S");
      type myElectricPotential = Real(final quantity = "ElectricPotential", final unit = "V");
      annotation (
        Icon(graphics = {Text(extent = {{-78, 60}, {72, -52}}, lineColor = {28, 108, 200}, fontName = "Segoe Print", textString = "U")}));
    end Units;

    package Types
      extends PowerSysPro.Icons.myPackage;
      type myPerUnit = Real(unit = "1");
      annotation (
        Icon(graphics = {Text(extent = {{-78, 62}, {72, -50}}, lineColor = {28, 108, 200}, fontName = "Segoe Print", textString = "T")}));
    end Types;

    package Icons
      extends PowerSysPro.Icons.myPackage;

      record myDataSet "Data icon"
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(origin = {0.0, -25.0}, lineColor = {64, 64, 64}, fillColor = {255, 215, 136}, fillPattern = FillPattern.Solid, extent = {{-100.0, -75.0}, {100.0, 75.0}}, radius = 25.0), Line(points = {{-100.0, 0.0}, {100.0, 0.0}}, color = {64, 64, 64}), Line(origin = {0.0, -50.0}, points = {{-100.0, 0.0}, {100.0, 0.0}}, color = {64, 64, 64})}),
          Documentation(info = "<html>
<p>
This icon is indicates a record.
</p>
</html>"));
      end myDataSet;

      model myBattery "Icon for cells and stacks"
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false), graphics={  Polygon(points = {{-90, 30}, {-100, 30}, {-110, 10}, {-110, -10}, {-100, -30}, {-90, -30}, {-90, 30}}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-90, 60}, {90, -60}}, lineColor = {0, 0, 255}, radius = 10), Text(extent = {{-150, 70}, {150, 110}}, textColor = {0, 0, 255}, textString = "%name"), Rectangle(extent = {{90, 40}, {110, -40}}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Rectangle(extent = DynamicSelect({{70, -40}, {-70, 40}}, {{70, -40}, {70 - 140 * displaySOC, 40}}), lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid)}));
      end myBattery;
      annotation (
        Icon(graphics = {Text(extent = {{-78, 60}, {72, -52}}, lineColor = {28, 108, 200}, fontName = "Segoe Print", textString = "I")}));
    end Icons;

    package Utilities
      extends PowerSysPro.Icons.myPackage;

      block PulseSeries "Series of pulses"
        import Modelica.Math.BooleanVectors.oneTrue;
        import Modelica.Units.SI;
        parameter Real amplitude1 = 1 "Amplitude of 1st pulse series";
        parameter Integer n1(min = 0) = 1 "Number of pulses of 1st series";
        parameter SI.Time T1 "Length of pulses of 1st series";
        parameter SI.Time Tp1 "Pause between pulses of 1st series";
        parameter Real amplitude2 = -amplitude1 "Amplitude of 2nd pulse series";
        parameter Integer n2(min = 0) = 1 "Number of pulses of 2nd series";
        parameter SI.Time T2 = T1 "Length of pulses of 2nd series";
        parameter SI.Time Tp2 = Tp1 "Pause between pulses of 1st series";
        parameter SI.Time Tp "Pause between the two series";
        extends Modelica.Blocks.Interfaces.SignalSource;
        parameter Real k(start = 1, unit = "1") "Gain value multiplied with input signal";
      protected
        parameter SI.Time Tstart1[n1] = {startTime + (k - 1) * (T1 + Tp1) for k in 1:n1};
        parameter SI.Time Tstart2[n1] = {startTime + n1 * (T1 + Tp1) + Tp + (k - 1) * (T2 + Tp2) for k in 1:n2};
        Boolean on1, on2;
      equation
        on1 = oneTrue({time >= Tstart1[k] and time < Tstart1[k] + T1 for k in 1:n1});
        on2 = oneTrue({time >= Tstart2[k] and time < Tstart2[k] + T2 for k in 1:n1});
        y = k * (offset + (if on1 then amplitude1 elseif on2 then amplitude2 else 0));
        annotation (
          Icon(graphics = {Line(points = {{-100, 0}, {-80, 0}}, color = {0, 0, 0}, pattern = LinePattern.Dash), Line(points = {{-10, 0}, {-10, -60}, {10, -60}, {10, 0}, {20, 0}}, color = {0, 0, 0}), Line(points = {{-50, 0}, {-50, 60}, {-40, 60}, {-40, 0}, {-20, 0}}, color = {0, 0, 0}), Line(points = {{-20, 0}, {-10, 0}}, color = {0, 0, 0}, pattern = LinePattern.Dash), Line(points = {{-80, 0}, {-80, 60}, {-70, 60}, {-70, 0}, {-50, 0}}, color = {0, 0, 0}), Line(points = {{20, 0}, {20, -60}, {40, -60}, {40, 0}, {50, 0}}, color = {0, 0, 0}), Line(points = {{50, 0}, {50, -60}, {70, -60}, {70, 0}, {80, 0}}, color = {0, 0, 0}), Line(points = {{80, 0}, {100, 0}}, color = {0, 0, 0}, pattern = LinePattern.Dash)}),
          Documentation(info = "<html>
<p>
Starting at <code>time = startTime</code>, first a series of <code>n1</code> pulses of <code>amplitude1</code> with length <code>T1</code> and pause after each pulse <code>Tp1</code> is issued.<br>
Then, after a pause <code>Tp</code>, a series of <code>n2</code> pulses of <code>amplitude2</code> with length <code>T2</code> and pause after each pulse <code>Tp2</code> is issued.
</p>
</html>"));
      end PulseSeries;
      annotation (
        Icon(graphics = {Polygon(origin = {5.383, -4.1418}, rotation = 45.0, fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-15.0, 93.333}, {-15.0, 68.333}, {0.0, 58.333}, {15.0, 68.333}, {15.0, 93.333}, {20.0, 93.333}, {25.0, 83.333}, {25.0, 58.333}, {10.0, 43.333}, {10.0, -41.667}, {25.0, -56.667}, {25.0, -76.667}, {10.0, -91.667}, {0.0, -91.667}, {0.0, -81.667}, {5.0, -81.667}, {15.0, -71.667}, {15.0, -61.667}, {5.0, -51.667}, {-5.0, -51.667}, {-15.0, -61.667}, {-15.0, -71.667}, {-5.0, -81.667}, {0.0, -81.667}, {0.0, -91.667}, {-10.0, -91.667}, {-25.0, -76.667}, {-25.0, -56.667}, {-10.0, -41.667}, {-10.0, 43.333}, {-25.0, 58.333}, {-25.0, 83.333}, {-20.0, 93.333}}), Polygon(origin = {14.102, 5.218}, rotation = -45.0, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-15.0, 87.273}, {15.0, 87.273}, {20.0, 82.273}, {20.0, 27.273}, {10.0, 17.273}, {10.0, 7.273}, {20.0, 2.273}, {20.0, -2.727}, {5.0, -2.727}, {5.0, -77.727}, {10.0, -87.727}, {5.0, -112.727}, {-5.0, -112.727}, {-10.0, -87.727}, {-5.0, -77.727}, {-5.0, -2.727}, {-20.0, -2.727}, {-20.0, 2.273}, {-10.0, 7.273}, {-10.0, 17.273}, {-20.0, 27.273}, {-20.0, 82.273}})}));
    end Utilities;
    annotation (
      Icon(graphics={  Text(extent = {{-132, 60}, {152, -58}}, lineColor = {28, 108, 200}, fontName = "Segoe Print", textString = "Ba")}));
  end Batteries;

  package Regulations "Regulations and controls"
    extends PowerSysPro.Icons.myPackage;

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
        Icon(graphics = {Text(origin = {-140.43, 18.67}, lineColor = {28, 108, 200}, extent = {{-53.57, 7.33}, {331.14, -14.66}}, textStyle = {TextStyle.Bold}, textString = "RegU")}),
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
        Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Text(origin = {-136.43, 18.67}, lineColor = {28, 108, 200}, extent = {{-53.57, 7.33}, {331.14, -14.66}}, textStyle = {TextStyle.Bold}, textString = "Q=f(U)")}),
        Diagram(coordinateSystem(preserveAspectRatio = false)),
        Documentation(info = "<html>
    <p>This regulation is derived from document Enedis-NOI-RES_60E published by ENEDIS. A diagram of the Q=f(U) law is shown in the next figure:</p>
    <p>  </p>
    <figure> <img src=\"modelica://PowerSysPro/Resources/Images/Q=f(U).png\"></figure>
</html>"));
    end myQfU;
    annotation (
      Icon(graphics={  Text(extent = {{-132, 60}, {152, -58}}, lineColor = {28, 108, 200}, fontName = "Segoe Print", textString = "R")}));
  end Regulations;

  package Buses "Bus for causal connections"
    extends PowerSysPro.Icons.myPackage;

    model myCausalBusVInput "Causal bus with voltage as input"
      extends Icons.myBus;
      Interfaces.myAcausalTerminal terminal "Terminal of the 1-port node" annotation (
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
      i_re = -terminal.i.re;
      i_im = -terminal.i.im;
      annotation (
        Documentation(info = "<html>
    <p> Causal input connector, with complex voltage as input and complex current as output.</p>    
    </html>"),
        Icon(graphics = {Text(origin = {73, 97}, lineColor = {0, 0, 255}, extent = {{-131, 11}, {131, -11}}, textString = "%name"), Text(extent = {{-10, -12}, {72, -52}}, lineColor = {0, 0, 0}, textStyle = {TextStyle.Bold}, textString = "~")}));
    end myCausalBusVInput;

    model myCausalBusVOutput "Causal bus with voltage as output"
      extends Icons.myBus;
      Interfaces.myAcausalTerminal terminal "Terminal of the 1-port node" annotation (
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
      i_re = terminal.i.re;
      i_im = terminal.i.im;
      annotation (
        Documentation(info = "<html>
    <p> Causal output connector, with complex voltage as output and complex current as input.</p>
    </html>"),
        Icon(graphics = {Text(origin = {-79, 97}, lineColor = {0, 0, 255}, extent = {{-131, 11}, {131, -11}}, textString = "%name"), Text(extent = {{-70, -10}, {12, -50}}, lineColor = {0, 0, 0}, textStyle = {TextStyle.Bold}, textString = "~")}));
    end myCausalBusVOutput;
    annotation (
      Icon(coordinateSystem(grid = {0.1, 0.1}), graphics={  Text(extent = {{-78, 52.9}, {72, -59.1}}, lineColor = {28, 108, 200}, fontName = "Segoe Print", textString = "Bu")}),
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
      Interfaces.myAcausalTerminal terminal "Terminal of the 1-port node" annotation (
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
        Icon(coordinateSystem(preserveAspectRatio = false, initialScale = 0.1), graphics = {Text(extent = {{-178, -4}, {102, -38}}, lineColor = {0, 0, 255}, lineThickness = 0.5, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, textString = "V")}));
    end VoltMeter;

    model AmMeter "Ideal ammeter"
      extends Icons.mySensor;
      Interfaces.myAcausalTerminal terminalA "Terminal A of the 2-port node" annotation (
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Interfaces.myAcausalTerminal terminalB "Terminal B of the 2-port node" annotation (
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Imes(unit = "A", displayUnit = "A") "Measured flowing current at port A" annotation (
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-40, 90}), iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {0, 90})));
    equation
      Imes = CM.abs(terminalA.i);
      terminalB.v = terminalA.v;
      terminalA.i + terminalB.i = Complex(0);
      annotation (
        Documentation(info = "<html>
    <p>This equipement measures the current flowing through the two connected terminals.</p>
    </html>"),
        Icon(coordinateSystem(preserveAspectRatio = false, initialScale = 0.1), graphics = {Text(extent = {{-178, -4}, {102, -38}}, lineColor = {0, 0, 255}, lineThickness = 0.5, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, textString = "A"), Polygon(origin = {40, -83}, fillColor = {70, 70, 70}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{0, 19}, {40, 0}, {0, -21}, {0, 19}}), Rectangle(fillColor = {70, 70, 70}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-70, -78}, {40, -88}})}));
    end AmMeter;

    model WattMeter "Versatile sensor"
      extends Icons.mySensor;
      Interfaces.myAcausalTerminal terminalA "Terminal A of the 2-port node" annotation (
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Interfaces.myAcausalTerminal terminalB "Terminal B of the 2-port node" annotation (
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Pmes(unit = "W", displayUnit = "kW") "Measured active flowing power at port A" annotation (
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-70, 90}), iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-70, 90})));
      Modelica.Blocks.Interfaces.RealOutput Qmes(unit = "var", displayUnit = "kvar") "Measured reactive flowing power at port A" annotation (
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {0, 90}), iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {0, 90})));
      Modelica.Blocks.Interfaces.RealOutput Smes(unit = "VA", displayUnit = "kVA") "Measured apparent flowing power at port A" annotation (
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {70, 90}), iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {70, 90})));
      Types.myCurrent Imes = CM.abs(terminalA.i) "Current flowing the source";
      Types.myVoltage Umes = sqrt(3) * CM.abs(terminalA.v) "Voltage at the source";
    equation
      Complex(Pmes, Qmes) = 3 * terminalA.v * CM.conj(terminalA.i);
      Smes = sqrt(3) * Imes * Umes;
      terminalB.v = terminalA.v;
      terminalA.i + terminalB.i = Complex(0);
      annotation (
        Documentation(info = "<html>
    <p>This equipement measures the active, reactive and apparent power flowing through the two connected terminals.</p>
    </html>"),
        Icon(coordinateSystem(preserveAspectRatio = false, initialScale = 0.1), graphics = {Text(extent = {{-178, -4}, {102, -38}}, lineColor = {0, 0, 255}, lineThickness = 0.5, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, textString = "W"), Polygon(origin = {40, -83}, fillColor = {70, 70, 70}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{0, 19}, {40, 0}, {0, -21}, {0, 19}}), Rectangle(fillColor = {70, 70, 70}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-70, -78}, {40, -88}})}));
    end WattMeter;

    model DiffMeter "Drop sensor"
      extends Icons.mySensor;
      Modelica.Blocks.Interfaces.RealOutput deltaP(unit = "W", displayUnit = "kW") "Measured active power drop from port A to port B";
      Modelica.Blocks.Interfaces.RealOutput deltaQ(unit = "var", displayUnit = "kvar") "Measured reactive power drop from port A to port B";
      Modelica.Blocks.Interfaces.RealOutput deltaS(unit = "VA", displayUnit = "kVA") "Measured apparent power drop from port A to port B";
      Modelica.Blocks.Interfaces.RealInput PA(unit = "W", displayUnit = "kW") "Measured active flowing power at port A" annotation (
        Placement(transformation(extent = {{-132, 44}, {-100, 76}}), iconTransformation(extent = {{-132, 44}, {-100, 76}})));
      Modelica.Blocks.Interfaces.RealInput QA(unit = "var", displayUnit = "kvar") "Measured reactive flowing power at port A" annotation (
        Placement(transformation(extent = {{-132, -16}, {-100, 16}}), iconTransformation(extent = {{-132, -16}, {-100, 16}})));
      Modelica.Blocks.Interfaces.RealInput SA(unit = "VA", displayUnit = "kVA") "Measured apparent flowing power at port A" annotation (
        Placement(transformation(extent = {{-16, -16}, {16, 16}}, rotation = 0, origin = {-116, -60}), iconTransformation(extent = {{-16, -16}, {16, 16}}, rotation = 0, origin = {-116, -60})));
      Modelica.Blocks.Interfaces.RealInput PB(unit = "W", displayUnit = "kW") "Measured active flowing power at port B" annotation (
        Placement(transformation(extent = {{-16, -16}, {16, 16}}, rotation = 180, origin = {116, 60}), iconTransformation(extent = {{-16, -16}, {16, 16}}, rotation = 180, origin = {116, 60})));
      Modelica.Blocks.Interfaces.RealInput QB(unit = "var", displayUnit = "kvar") "Measured reactive flowing power at port B" annotation (
        Placement(transformation(extent = {{-16, -16}, {16, 16}}, rotation = 180, origin = {116, 0}), iconTransformation(extent = {{102, -16}, {134, 16}})));
      Modelica.Blocks.Interfaces.RealInput SB(unit = "VA", displayUnit = "kVA") "Measured apparent flowing power at port B" annotation (
        Placement(transformation(extent = {{-16, -16}, {16, 16}}, rotation = 180, origin = {116, -60}), iconTransformation(extent = {{-16, -16}, {16, 16}}, rotation = 180, origin = {116, -60})));
    equation
      deltaP = PA - PB;
      deltaQ = QA - QB;
      deltaS = SA - SB;
      annotation (
        Documentation(info = "<html>
    <p>This equipement measures the difference of voltage, current, and apparent power between its two connected terminals.</p>
    </html>"),
        Icon(graphics = {Polygon(origin = {40, -83}, fillColor = {70, 70, 70}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{0, 19}, {40, 0}, {0, -21}, {0, 19}}), Rectangle(fillColor = {70, 70, 70}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-70, -78}, {40, -88}}), Text(extent = {{-140, 128}, {140, 94}}, lineColor = {0, 0, 255}, lineThickness = 0.5, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, textString = "delta")}));
    end DiffMeter;
    annotation (
      Icon(coordinateSystem(grid = {0.1, 0.1}), graphics={  Text(extent = {{-78, 52.9}, {72, -59.1}}, lineColor = {28, 108, 200}, fontName = "Segoe Print", textString = "S")}),
      Diagram(coordinateSystem(extent = {{-200, -100}, {200, 100}})),
      Documentation(info = "<html>
    <p>Ideal sensors are proposed to measure electric states.</p>
    <p>One of them measures the drop of electric states between two connected terminals.</p>
</body></html>"));
  end Sensors;

  package Interfaces "Interfaces for connecting the components"
    extends Icons.myInterfacesPackage;

    connector myAcausalTerminal "Non-causal terminal for phasor-based AC connections"
      Types.myComplexVoltage v "Phase-to-ground voltage phasor";
      flow Types.myComplexCurrent i "Current phasor";
      annotation (
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 1, grid = {2, 2}), graphics={  Rectangle(origin = {92, 3}, fillColor = {85, 170, 255}, fillPattern = FillPattern.Solid, extent = {{-192, 97}, {8, -103}})}),
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
    extends PowerSysPro.Icons.myPackage;
    // prefix Functions. before asin is to be removed (crash with OM Windows: ticket #6389)
    final constant Real pi = 2 * asin(1.0) "Pi number";

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
      //external "builtin" y = cos(u);
    algorithm
      y := Modelica.Math.cos(u);
    end cos;

    function sin "Sine"
      extends Icons.myFunction;
      input Types.myAngle u "Independent variable";
      output Real y "Dependent variable y=sin(u)";
      //external "builtin" y = sin(u);
    algorithm
      y := Modelica.Math.sin(u);
    end sin;

    function asin "Inverse sine (-1 <= u <= 1)"
      extends Icons.myFunction;
      input Real u "Independent variable";
      output Types.myAngle y "Dependent variable y=asin(u)";
      //external "builtin" y = asin(u);
    algorithm
      y := Modelica.Math.asin(u);
    end asin;
    annotation (
      Icon(graphics={  Text(extent = {{-72, 58.9}, {78, -53.1}}, lineColor = {28, 108, 200}, fontName = "Segoe Print", textString = "F")}),
      Documentation(info = "<html>
    <p>Rewrite MSL functions for portability needs.</p>
</body></html>"));
  end Functions;

  package Types "Domain-specific type definitions"
    extends PowerSysPro.Icons.myPackage;
    type myVoltage = Real(unit = "V", displayUnit = "kV");
    type myCurrent = Real(unit = "A", displayUnit = "A");
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
    operator record myComplexVoltage = Complex(redeclare myVoltage re "Imaginary part of complex voltage", redeclare myVoltage im "Real part of complex voltage") "Complex voltage";
    operator record myComplexCurrent = Complex(redeclare myCurrent re "Real part of complex current", redeclare myCurrent im "Imaginary part of complex current") "Complex current";
    operator record myComplexAdmittance = Complex(redeclare myConductance re "Real part of complex admittance (conductance)", redeclare mySusceptance im "Imaginary part of complex admittance (susceptance)") "Complex admittance";
    operator record myComplexImpedance = Complex(redeclare myResistance re "Real part of complex impedance (resistance)", redeclare myReactance im "Imaginary part of complex impedance (reactance)") "Complex impedance";
    operator record myComplexPerUnit = Complex(re(unit = "1"), im(unit = "1")) "Complexe per unit";
    annotation (
      Documentation(info = "<html>
    <p>This package gathers dedicated icons for all models in this library.</p>
</body></html>"),
      Icon(graphics={  Text(extent = {{-78, 62}, {72, -50}}, lineColor = {28, 108, 200}, fontName = "Segoe Print", textString = "T")}));
  end Types;

  package Icons "Icons for the library models"
    extends PowerSysPro.Icons.myPackage;

    model mySource "Icon for fixed source node"
      annotation (
        Icon(graphics={  Ellipse(origin = {0, -1}, extent = {{-60, 61}, {60, -59}}, endAngle = 360), Text(origin = {0, 96}, lineColor = {0, 0, 255}, extent = {{-118, 12}, {118, -12}}, textString = "%name"), Text(origin = {-138.43, -89.33}, lineColor = {28, 108, 200}, extent = {{-53.57, 7.33}, {331.14, -14.66}}, textStyle = {TextStyle.Bold}, textString = "%UNom"), Line(origin = {6.7702, 12.33}, points = {{-51.4286, -11.4286}, {-44.9714, 8.11429}, {-40.8571, 18.9143}, {-37.2, 26.5143}, {-33.9429, 31.2}, {-30.7429, 33.7714}, {-27.5429, 34.1714}, {-24.3429, 32.3429}, {-21.0857, 28.4}, {-17.8857, 22.5143}, {-14.2286, 13.7714}, {-9.61714, 0.685714}, {0.0571429, -29.0286}, {4.17143, -40.1143}, {7.82857, -48.1143}, {11.0286, -53.2}, {14.2857, -56.2286}, {17.4857, -57.1429}, {20.6857, -55.7714}, {23.9429, -52.2857}, {27.1429, -46.8}, {30.8, -38.4}, {35.4286, -25.6}, {40, -11.4286}}, smooth = Smooth.Bezier)}, coordinateSystem(initialScale = 0.1)));
    end mySource;

    model myPV "Icon for PV node"
      annotation (
        Icon(graphics={  Ellipse(origin = {0, -1}, extent = {{-60, 61}, {60, -59}}, endAngle = 360), Text(origin = {0, 96}, lineColor = {0, 0, 255}, extent = {{-118, 12}, {118, -12}}, textString = "%name"), Text(origin = {-138.43, -89.33}, lineColor = {28, 108, 200}, extent = {{-53.57, 7.33}, {331.14, -14.66}}, textStyle = {TextStyle.Bold}, textString = "%UNom")}, coordinateSystem(initialScale = 0.1)));
    end myPV;

    model myBattery "Icon for fixed source node"
      annotation (
        Icon(graphics={  Ellipse(origin = {0, -1}, extent = {{-60, 61}, {60, -59}}), Text(origin = {0, 96}, lineColor = {0, 0, 255}, extent = {{-118, 12}, {118, -12}}, textString = "%name"), Text(origin = {-138.43, -89.33}, lineColor = {28, 108, 200}, extent = {{-53.57, 7.33}, {331.14, -14.66}}, textString = "%UNom", textStyle = {TextStyle.Bold}), Rectangle(origin = {30, -32}, extent = {{-1, 6}, {3, -6}}), Rectangle(origin = {-5, -32}, fillColor = {255, 255, 255}, extent = {{-33, -12}, {33, 12}}), Rectangle(origin = {-18, -32}, extent = {{-4, 8}, {4, -8}}), Rectangle(origin = {20, -32}, extent = {{-4, 8}, {4, -8}}), Rectangle(origin = {8, -32}, extent = {{-4, 8}, {4, -8}}), Rectangle(origin = {-6, -32}, extent = {{-4, 8}, {4, -8}}), Rectangle(origin = {-30, -32}, extent = {{-4, 8}, {4, -8}})}, coordinateSystem(initialScale = 0.1)));
    end myBattery;

    model myLoad "Icon for load node"
      annotation (
        Icon(graphics={  Polygon(origin = {70, 0}, rotation = 90, fillPattern = FillPattern.Solid, points = {{-40, 30}, {40, 30}, {0, -30}, {-40, 30}}), Text(origin = {62, 67}, lineColor = {0, 0, 255}, extent = {{-124, 11}, {124, -11}}, textString = "%name"), Line(points = {{0, 0}, {40, 0}})}, coordinateSystem(initialScale = 0.1)));
    end myLoad;

    model myCapacitorBank "Icon for capacitor bank"
      annotation (
        Icon(graphics={  Polygon(origin = {70, 0}, fillPattern = FillPattern.Solid, points = {{-40, 30}, {40, 30}, {0, -30}, {-40, 30}}, rotation = 90), Rectangle(origin = {29, 0}, fillPattern = FillPattern.Solid, extent = {{-20, 1}, {20, -1}}, rotation = 270), Line(origin = {32, 0}, points = {{0, 8}, {1.07002e-15, -2}}, rotation = 270), Line(origin = {12, 0}, points = {{-4.88625e-16, 6}, {7.34788e-16, -6}}, rotation = 270), Text(origin = {62, 67}, lineColor = {0, 0, 255}, extent = {{-124, 11}, {124, -11}}, textString = "%name"), Rectangle(origin = {19, 0}, fillPattern = FillPattern.Solid, extent = {{-20, 1}, {20, -1}}, rotation = 270)}, coordinateSystem(initialScale = 0.1)));
    end myCapacitorBank;

    model myLine "Icon for line"
      annotation (
        Icon(graphics={  Rectangle(origin = {-1, -1}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-59, 11}, {61, -11}}), Line(origin = {-80, 0}, points = {{-20, 0}, {20, 0}}), Line(origin = {80, 0}, points = {{-20, 0}, {20, 0}}), Text(origin = {0, 64}, lineColor = {0, 0, 255}, extent = {{-118, 12}, {118, -12}}, textString = "%name"), Text(origin = {-108.435, -55.3303}, lineColor = {28, 108, 200}, extent = {{-53.5654, 7.33031}, {265.143, -16.6612}}, textStyle = {TextStyle.Bold}, textString = "%Imax")}, coordinateSystem(initialScale = 0.1)));
    end myLine;

    model myTransformer "Icon for transformer"
      annotation (
        Icon(graphics={  Ellipse(origin = {-15, -7}, extent = {{-45, 47}, {35, -33}}, endAngle = 360), Ellipse(origin = {3, -9}, extent = {{57, 49}, {-23, -31}}, endAngle = 360), Line(origin = {-80, 0}, points = {{-20, 0}, {20, 0}, {20, 0}}), Line(origin = {80, 0}, points = {{-20, 0}, {20, 0}}), Text(origin = {-1, 94}, lineColor = {0, 0, 255}, extent = {{-179, 14}, {179, -14}}, textString = "%name"), Text(origin = {-91.9974, -90.6665}, lineColor = {28, 108, 200}, extent = {{-92.004, 6.66649}, {281.997, -11.3335}}, textStyle = {TextStyle.Bold}, textString = "%SNom")}, coordinateSystem(initialScale = 0.1)));
    end myTransformer;

    model myBreaker "Icon for breaker"
      annotation (
        Icon(graphics={  Line(points = {{90, 0}, {40, 0}}, color = {0, 0, 0}, thickness = 0.5), Text(origin = {0, 68}, lineColor = {0, 0, 255}, extent = {{-118, 12}, {118, -12}}, textString = "%name"), Line(points = {{-40, 0}, {-90, 0}}, color = {0, 0, 0}, thickness = 0.5)}));
    end myBreaker;

    model myBus "Icon for causal bus"
      annotation (
        Icon(graphics = {Rectangle(origin = {1, 25}, fillPattern = FillPattern.Solid, extent = {{-5, 75}, {5, -125}})}, coordinateSystem(initialScale = 0.1)));
    end myBus;

    model myRegulation "Icon for tape changer"
      annotation (
        Icon(coordinateSystem(initialScale = 0.2), graphics = {Rectangle(extent = {{-100, 40}, {98, -50}}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, pattern = LinePattern.None), Line(points = {{-76, -18}, {-48, 24}, {-12, -50}, {20, -4}, {60, -30}, {82, 20}}, color = {0, 0, 255}, smooth = Smooth.None), Line(points = {{-88, 0}, {88, 0}}, color = {0, 0, 0}), Line(points = {{-86, 32}, {90, 32}}, color = {192, 192, 192}, thickness = 1), Line(points = {{-88, -30}, {88, -30}}, color = {192, 192, 192}, thickness = 1)}));
    end myRegulation;

    model myFault "Icon for fault"
      annotation (
        Icon(graphics={  Rectangle(origin = {-1, -1}, fillColor = {238, 46, 47}, fillPattern = FillPattern.Solid, extent = {{-59, 11}, {61, -11}}, lineColor = {238, 46, 47})}, coordinateSystem(initialScale = 0.1)));
    end myFault;

    model mySubNetwork "Icon for sub-network"
      annotation (
        Icon(coordinateSystem(initialScale = 0.1), graphics={  Rectangle(extent = {{-80, 20}, {78, -20}}, lineColor = {28, 108, 200}, pattern = LinePattern.None), Rectangle(extent = {{-58, 40}, {60, -20}}, lineColor = {28, 108, 200}, pattern = LinePattern.None), Rectangle(origin = {-1.66795, 0}, extent = {{-98.332, 40}, {101.668, -40}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillColor = {255, 255, 255}, fillPattern = FillPattern.Sphere, pattern = LinePattern.Dash)}));
    end mySubNetwork;

    model mySensor "Icon for sensor"
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Ellipse(fillColor = {245, 245, 245}, fillPattern = FillPattern.Solid, extent = {{-70, -68}, {70, 72}}), Line(points = {{-37.6, 15.7}, {-65.8, 25.9}}), Line(points = {{-22.9, 34.8}, {-40.2, 59.3}}), Line(points = {{0, 72}, {0, 42}}), Line(points = {{22.9, 34.8}, {40.2, 59.3}}), Line(points = {{37.6, 15.7}, {65.8, 25.9}}), Polygon(origin = {0, 2}, rotation = -17.5, fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-5.0, 0.0}, {-2.0, 60.0}, {0.0, 65.0}, {2.0, 60.0}, {5.0, 0.0}}), Ellipse(lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, extent = {{-12, -10}, {12, 14}}), Ellipse(fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-7, -5}, {7, 9}}), Text(extent = {{-29, -9}, {30, -68}}, lineColor = {0, 0, 0}, textString = "I"), Line(points = {{-70, 0}, {-100, 0}}, color = {0, 0, 255}, thickness = 0.5), Line(points = {{70, 0}, {100, 0}}, color = {0, 0, 255}, thickness = 0.5), Text(origin = {-2, -133}, lineColor = {0, 0, 255}, extent = {{-124, 11}, {124, -11}}, textString = "%name")}),
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
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Text(textColor = {0, 0, 255}, extent = {{-150, 105}, {150, 145}}, textString = "%name"), Ellipse(lineColor = {108, 88, 49}, fillColor = {255, 215, 136}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Text(textColor = {108, 88, 49}, extent = {{-90.0, -90.0}, {90.0, 90.0}}, textString = "f")}),
        Documentation(info = "<html>
<p>This icon indicates functions.</p>
</html>"));
    end myFunction;

    partial model myExample "Icon for runnable tests and examples"
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Ellipse(lineColor = {75, 138, 73}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Polygon(lineColor = {0, 0, 255}, fillColor = {75, 138, 73}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-36, 60}, {64, 0}, {-36, -60}, {-36, 60}})}),
        Documentation(info = "<html>
<p>This icon indicates an example. The play button suggests that the example can be executed.</p>
</html>"));
    end myExample;

    partial record myRecord "Icon for records"
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Text(textColor = {0, 0, 255}, extent = {{-150, 60}, {150, 100}}, textString = "%name"), Rectangle(origin = {0.0, -25.0}, lineColor = {64, 64, 64}, fillColor = {255, 215, 136}, fillPattern = FillPattern.Solid, extent = {{-100.0, -75.0}, {100.0, 75.0}}, radius = 25.0), Line(points = {{-100.0, 0.0}, {100.0, 0.0}}, color = {64, 64, 64}), Line(origin = {0.0, -50.0}, points = {{-100.0, 0.0}, {100.0, 0.0}}, color = {64, 64, 64}), Line(origin = {0.0, -25.0}, points = {{0.0, 75.0}, {0.0, -75.0}}, color = {64, 64, 64})}),
        Documentation(info = "<html>
<p>
This icon is indicates a record.
</p>
</html>"));
    end myRecord;

    partial package myPackage "Icon for standard packages"
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-100.0, -100.0}, {100.0, 100.0}}, radius = 25.0), Rectangle(lineColor = {128, 128, 128}, extent = {{-100.0, -100.0}, {100.0, 100.0}}, radius = 25.0)}),
        Documentation(info = "<html>
<p>Standard package icon.</p>
</html>"));
    end myPackage;

    partial package myBasesPackage "Icon for packages containing base classes"
      extends Icons.myPackage;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Ellipse(extent = {{-30.0, -30.0}, {30.0, 30.0}}, lineColor = {128, 128, 128}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid)}),
        Documentation(info = "<html>
<p>This icon shall be used for a package/library that contains base models and classes, respectively.</p>
</html>"));
    end myBasesPackage;

    partial package mySensorsPackage "Icon for packages containing sensors"
      extends PowerSysPro.Icons.myPackage;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Ellipse(origin = {0.0, -30.0}, fillColor = {255, 255, 255}, extent = {{-90.0, -90.0}, {90.0, 90.0}}, startAngle = 20.0, endAngle = 160.0), Ellipse(origin = {0.0, -30.0}, fillColor = {128, 128, 128}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-20.0, -20.0}, {20.0, 20.0}}), Line(origin = {0.0, -30.0}, points = {{0.0, 60.0}, {0.0, 90.0}}), Ellipse(origin = {-0.0, -30.0}, fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-10.0, -10.0}, {10.0, 10.0}}), Polygon(origin = {-0.0, -30.0}, rotation = -35.0, fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-7.0, 0.0}, {-3.0, 85.0}, {0.0, 90.0}, {3.0, 85.0}, {7.0, 0.0}})}),
        Documentation(info = "<html>
<p>This icon indicates a package containing sensors.</p>
</html>"));
    end mySensorsPackage;

    partial package myInterfacesPackage "Icon for packages containing interfaces"
      extends Icons.myPackage;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Polygon(origin = {20.0, 0.0}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-10.0, 70.0}, {10.0, 70.0}, {40.0, 20.0}, {80.0, 20.0}, {80.0, -20.0}, {40.0, -20.0}, {10.0, -70.0}, {-10.0, -70.0}}), Polygon(fillColor = {102, 102, 102}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-100.0, 20.0}, {-60.0, 20.0}, {-30.0, 70.0}, {-10.0, 70.0}, {-10.0, -70.0}, {-30.0, -70.0}, {-60.0, -20.0}, {-100.0, -20.0}})}),
        Documentation(info = "<html>
<p>This icon indicates packages containing interfaces.</p>
</html>"));
    end myInterfacesPackage;

    partial package myExamplesPackage "Icon for packages containing tests and examples"
      extends Icons.myPackage;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Polygon(origin = {8.0, 14.0}, lineColor = {78, 138, 73}, fillColor = {78, 138, 73}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-58.0, 46.0}, {42.0, -14.0}, {-58.0, -74.0}, {-58.0, 46.0}})}),
        Documentation(info = "<html>
<p>This icon indicates a package that contains executable examples.</p>
</html>"));
    end myExamplesPackage;

    partial class myInformation "Icon for general information packages"
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Ellipse(lineColor = {75, 138, 73}, fillColor = {75, 138, 73}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100.0, -100.0}, {100.0, 100.0}}), Polygon(origin = {-4.167, -15.0}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-15.833, 20.0}, {-15.833, 30.0}, {14.167, 40.0}, {24.167, 20.0}, {4.167, -30.0}, {14.167, -30.0}, {24.167, -30.0}, {24.167, -40.0}, {-5.833, -50.0}, {-15.833, -30.0}, {4.167, 20.0}, {-5.833, 20.0}}, smooth = Smooth.Bezier), Ellipse(origin = {7.5, 56.5}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-12.5, -12.5}, {12.5, 12.5}})}),
        Documentation(info = "<html>
<p>This icon indicates classes containing only documentation, intended for general description of, e.g., concepts and features of a package.</p>
</html>"));
    end myInformation;

    partial class myReleaseNotes "Icon for general information"
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Polygon(points = {{-80, -100}, {-80, 100}, {0, 100}, {0, 20}, {80, 20}, {80, -100}, {-80, -100}}, fillColor = {245, 245, 245}, fillPattern = FillPattern.Solid), Polygon(points = {{0, 100}, {80, 20}, {0, 20}, {0, 100}}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Solid), Line(points = {{2, -12}, {50, -12}}), Ellipse(extent = {{-56, 2}, {-28, -26}}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Solid), Line(points = {{2, -60}, {50, -60}}), Ellipse(extent = {{-56, -46}, {-28, -74}}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Solid)}),
        Documentation(info = "<html>
<p>This icon indicates release notes and the revision history of a library.</p>
</html>"));
    end myReleaseNotes;

    partial class myContact "Icon for contact information"
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-100, 70}, {100, -72}}, fillColor = {235, 235, 235}, fillPattern = FillPattern.Solid), Polygon(points = {{-100, -72}, {100, -72}, {0, 20}, {-100, -72}}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Solid), Polygon(points = {{22, 0}, {100, 70}, {100, -72}, {22, 0}}, fillColor = {235, 235, 235}, fillPattern = FillPattern.Solid), Polygon(points = {{-100, 70}, {100, 70}, {0, -20}, {-100, 70}}, fillColor = {241, 241, 241}, fillPattern = FillPattern.Solid)}),
        Documentation(info = "<html>
<p>This icon shall be used for the contact information of the library developers.</p>
</html>"));
    end myContact;
    annotation (
      Documentation(info = "<html>
    <p>This package contents dedicated icons for the basic components.</p>
    </body></html>"),
      Icon(graphics={  Text(extent = {{-76, 58}, {74, -54}}, lineColor = {28, 108, 200}, fontName = "Segoe Print", textString = "I")}));
  end Icons;

  package Tests "Tests package based on single component testing"
    extends Icons.myExamplesPackage;

    model OneSource "Testing a source"
      extends Icons.myExample;
      Components.mySource src(UNom = 10000, theta = 0.5235987755983) annotation (
        Placement(visible = true, transformation(origin = {-52, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Buses.myCausalBusVOutput Bout annotation (
        Placement(transformation(extent = {{-26, -16}, {-6, 4}})));
      Modelica.Blocks.Interfaces.RealInput i_re(start = 30) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 180, origin = {100, 24})));
      Modelica.Blocks.Interfaces.RealInput i_im(start = 40) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 180, origin = {100, -8})));
    equation
      connect(i_re, Bout.i_re) annotation (
        Line(points = {{100, 24}, {22, 24}, {22, 2}, {-14, 2}}, color = {0, 0, 127}));
      connect(src.terminal, Bout.terminal) annotation (
        Line(points = {{-52, -6}, {-19, -6}}, color = {0, 0, 0}));
      connect(i_im, Bout.i_im) annotation (
        Line(points = {{100, -8}, {22, -8}, {22, -2}, {-14, -2}}, color = {0, 0, 127}));
      annotation (
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-108, 24}, {34, 14}}, fontSize = 12, textString = "U = 10 kV, theta = 30°"), Text(lineColor = {28, 108, 200}, extent = {{-94, -20}, {106, -60}}, fontSize = 12, textString = "current in the source is 50 A
apparent power flowing the source is 866 kVA"), Text(lineColor = {28, 108, 200}, extent = {{10, 48}, {152, 38}}, fontSize = 12, textString = "i_re = 30, i_im = 40")}, coordinateSystem(initialScale = 0.1)),
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
        Diagram(graphics = {Text(extent = {{-106, -22}, {106, -64}}, lineColor = {28, 108, 200}, fontSize = 12, textString = "voltage at the load is 5 kV
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
      Modelica.Blocks.Interfaces.RealInput i_re(start = 30) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 180, origin = {104, 34})));
      Modelica.Blocks.Interfaces.RealInput i_im(start = 40) annotation (
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
        Diagram(graphics = {Text(extent = {{-58, -12}, {70, -62}}, lineColor = {28, 108, 200}, fontSize = 12, textString = "current in the line is 50 A and voltage is 5 kV 
no voltage drop as the line is perfect
apparent power flowing the line is 433 kVA"), Text(origin = {42.8, 22.3078}, lineColor = {28, 108, 200}, extent = {{-76.8, 19.6922}, {-12.8, 35.6922}}, fontSize = 12, textString = "perfect line:
R very low, and X=G=B=0"), Text(lineColor = {28, 108, 200}, extent = {{30, 60}, {172, 50}}, fontSize = 12, textString = "i_re = 30, i_im = 40"), Text(lineColor = {28, 108, 200}, extent = {{-208, 74}, {20, 66}}, fontSize = 12, textString = "v_re = 4000/sqrt(3), v_im = 3000/sqrt(3)")}, coordinateSystem(initialScale = 0.1)),
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
      Modelica.Blocks.Interfaces.RealInput i_re(start = 30) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 180, origin = {100, 40})));
      Modelica.Blocks.Interfaces.RealInput i_im(start = 40) annotation (
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
        Diagram(graphics = {Text(origin = {38.8, 18.3078}, lineColor = {28, 108, 200}, extent = {{-76.8, 19.6922}, {-12.8, 35.6922}}, fontSize = 12, textString = "resistive line:
R = 10 ohms, and X=G=B=0"), Text(extent = {{-58, -18}, {70, -68}}, lineColor = {28, 108, 200}, fontSize = 12, textString = "current in the line is 50 A and voltage is 5 kV 
voltage drop is 466 V
apparent power flowing the line is 433 kVA"), Text(lineColor = {28, 108, 200}, extent = {{26, 66}, {168, 56}}, fontSize = 12, textString = "i_re = 30, i_im = 40"), Text(lineColor = {28, 108, 200}, extent = {{-188, 64}, {-46, 54}}, fontSize = 12, textString = "v_re = 5000/sqrt(3), v_im = 0")}, coordinateSystem(initialScale = 0.1)),
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
      Modelica.Blocks.Interfaces.RealInput i_re(start = 30) annotation (
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 180, origin = {100, 42})));
      Modelica.Blocks.Interfaces.RealInput i_im(start = 40) annotation (
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
        Diagram(graphics = {Text(origin = {44.8, 20.3078}, lineColor = {28, 108, 200}, extent = {{-76.8, 19.6922}, {-12.8, 35.6922}}, fontSize = 12, textString = "perfect transformer:
R very low, and X=G=B=0
ratio=20/63"), Text(extent = {{-66, -14}, {62, -64}}, lineColor = {28, 108, 200}, fontSize = 12, textString = "voltage at port A is 63 kV and is 20 kV at port B
current in port A is 15.9 A and is 50 A in port B
apparent power flowing the transformer is 1732 kVA"), Text(lineColor = {28, 108, 200}, extent = {{26, 68}, {168, 58}}, fontSize = 12, textString = "i_re = 30, i_im = 40"), Text(lineColor = {28, 108, 200}, extent = {{-200, 66}, {-6, 62}}, fontSize = 12, textString = "v_re = 63000/sqrt(3), v_im = 0")}, coordinateSystem(initialScale = 0.1)),
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
      Modelica.Blocks.Interfaces.RealInput i_re(start = 50) annotation (
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
        Diagram(graphics = {Text(origin = {40.8, 24.3078}, lineColor = {28, 108, 200}, extent = {{-76.8, 19.6922}, {-12.8, 35.6922}}, fontSize = 12, textString = "resistive transformer:
R=10 ohms, and X=G=B=0
ratio=20/63"), Text(extent = {{-132, -8}, {142, -58}}, lineColor = {28, 108, 200}, fontSize = 12, textString = "voltage at port A is 63 kV
voltage drop at port B is 87 V bellow 20 kV
current in port A is 15.9 A and is 50 A in port B
apparent power flowing the transformer is 1732 kVA"), Text(lineColor = {28, 108, 200}, extent = {{24, 76}, {166, 66}}, fontSize = 12, textString = "i_re = 50, i_im = 0"), Text(lineColor = {28, 108, 200}, extent = {{-198, 70}, {-4, 66}}, fontSize = 12, textString = "v_re = 63000/sqrt(3), v_im = 0")}, coordinateSystem(initialScale = 0.1)),
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
        Diagram(coordinateSystem(preserveAspectRatio = false), graphics = {Text(lineColor = {28, 108, 200}, extent = {{28, 44}, {104, 22}}, fontSize = 12, textString = "B = 1 S"), Text(extent = {{-38, -22}, {36, -66}}, lineColor = {28, 108, 200}, fontSize = 12, textString = "voltage at the bank is 5 kV
current is 2887 A
apparent power flowing the bank is 25 MVA"), Text(lineColor = {28, 108, 200}, extent = {{-242, 74}, {62, 66}}, fontSize = 12, textString = "v_re = 4000/sqrt(3), v_im = 3000/sqrt(3)")}),
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
        Diagram(coordinateSystem(preserveAspectRatio = false), graphics = {Text(lineColor = {28, 108, 200}, extent = {{-94, 0}, {30, -74}}, fontSize = 12, horizontalAlignment = TextAlignment.Left, textString = "voltage evolution:
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
t=341 s"), Text(lineColor = {28, 108, 200}, extent = {{-60, 0}, {64, -74}}, fontSize = 12, horizontalAlignment = TextAlignment.Left, textString = "
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
U=20400 V"), Text(lineColor = {28, 108, 200}, extent = {{-2, -2}, {122, -76}}, fontSize = 12, horizontalAlignment = TextAlignment.Left, textString = "tap decrease at t=90 s,

tap increase at t=180 s, and t=190 s

tap decrease at t=310 s, t=320 s, t=330 s, and t=340 s"), Text(lineColor = {28, 108, 200}, extent = {{-138, 58}, {224, 28}}, fontSize = 12, horizontalAlignment = TextAlignment.Left, textString = "In this case, the starting voltage is in the normal range [underMinU, aboveMaxU]")}),
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
        Diagram(coordinateSystem(preserveAspectRatio = false), graphics = {Text(lineColor = {28, 108, 200}, extent = {{-94, 24}, {30, -50}}, fontSize = 12, horizontalAlignment = TextAlignment.Left, textString = "voltage evolution:
t=0 s 
t=300 s"), Text(lineColor = {28, 108, 200}, extent = {{-64, 20}, {60, -54}}, fontSize = 12, horizontalAlignment = TextAlignment.Left, textString = "U=21400 V
U=20400 V"), Text(lineColor = {28, 108, 200}, extent = {{-2, 20}, {122, -54}}, fontSize = 12, horizontalAlignment = TextAlignment.Left, textString = "tap decrease at t=60 s, 70 s, 80 s,
90 s, 100 s, 110 s, 120 s, and 130 s"), Text(lineColor = {28, 108, 200}, extent = {{-132, 58}, {230, 28}}, fontSize = 12, horizontalAlignment = TextAlignment.Left, textString = "In this case, the starting voltage is outside the normal range [underMinU, aboveMaxU]")}),
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
        Diagram(coordinateSystem(preserveAspectRatio = false), graphics = {Text(lineColor = {28, 108, 200}, extent = {{-68, 16}, {56, -58}}, fontSize = 12, horizontalAlignment = TextAlignment.Left, textString = "a variable reactive power is calculated
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
        Placement(visible = true, transformation(origin = {-36, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load(UNom = 10000, P = 4000, Q = 3000) annotation (
        Placement(visible = true, transformation(origin = {34, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(src.terminal, load.terminal) annotation (
        Line(points = {{-36, 20}, {4, 20}, {4, 19.97}, {33.99, 19.97}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-34, -10}, {40, -54}}, fontSize = 12, textString = "voltage is 10 kV
current is 0,289 A
apparent power flowing the components is 5 kVA"), Text(lineColor = {28, 108, 200}, extent = {{-30, 54}, {134, 28}}, fontSize = 12, textString = "P=4 kW and Q=3 kvar")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceOneLoad;

    model OneSourceOneLoadwithSensors
      extends Icons.myExample;
      Sensors.AmMeter amp annotation (
        Placement(transformation(extent = {{-10, 10}, {10, 30}})));
      Sensors.VoltMeter volt annotation (
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 0, origin = {0, 56})));
      Components.mySource src(UNom = 10000) annotation (
        Placement(visible = true, transformation(origin = {-36, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load(UNom = 10000, P = 4000, Q = 3000) annotation (
        Placement(visible = true, transformation(origin = {34, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Sensors.WattMeter watt annotation (
        Placement(transformation(extent = {{-10, -28}, {10, -8}})));
      Components.mySource src1(UNom = 10000) annotation (
        Placement(visible = true, transformation(origin = {-36, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load1(UNom = 10000, P = 4000, Q = 3000) annotation (
        Placement(visible = true, transformation(origin = {34, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(volt.terminal, amp.terminalA) annotation (
        Line(points = {{-10, 56}, {-10, 20}}, color = {0, 0, 0}));
      connect(src.terminal, amp.terminalA) annotation (
        Line(points = {{-36, 20}, {-10, 20}}, color = {0, 0, 0}));
      connect(amp.terminalB, load.terminal) annotation (
        Line(points = {{10, 20}, {33.96, 20.02}}, color = {0, 0, 0}));
      connect(src1.terminal, watt.terminalA) annotation (
        Line(points = {{-36, -18}, {-10, -18}}, color = {0, 0, 0}));
      connect(watt.terminalB, load1.terminal) annotation (
        Line(points = {{10, -18}, {33.96, -17.98}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-34, -40}, {40, -84}}, fontSize = 12, textString = "measured voltage is 10 kV
measured current is 0,289 A
measured apparent power is 5 kVA"), Text(lineColor = {28, 108, 200}, extent = {{-30, 54}, {134, 28}}, fontSize = 12, textString = "P=4 kW and Q=3 kvar")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceOneLoadwithSensors;

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
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{26, 42}, {102, 20}}, fontSize = 12, textString = "P=4 kW and Q=3 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-34, -20}, {40, -64}}, fontSize = 12, textString = "same results as previous model
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
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-22, 52}, {150, 26}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-36, -10}, {38, -54}}, fontSize = 12, textString = "source voltage is 10 kV
current in load is 0.289 A
voltage drop is 10 V in the resistive line")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceOneLineOneLoad;

    model TwoSourcesTwoLinesOneLoad
      extends Icons.myExample;
      Components.mySource src1(UNom = 20000) annotation (
        Placement(visible = true, transformation(origin = {-26, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load(UNom = 20000, P = 500000, Q = 150000) annotation (
        Placement(visible = true, transformation(origin = {36, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.mySource src2(UNom = 20000) annotation (
        Placement(visible = true, transformation(origin = {-26, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line1(UNom = 20000, Imax = 50, R = 1) annotation (
        Placement(visible = true, transformation(origin = {6, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line2(UNom = 20000, Imax = 50, R = 2) annotation (
        Placement(visible = true, transformation(origin = {6, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(src1.terminal, line1.terminalA) annotation (
        Line(points = {{-26, 30}, {-4, 30}}, color = {0, 0, 0}));
      connect(load.terminal, line1.terminalB) annotation (
        Line(points = {{35.99, 9.97}, {26, 9.97}, {26, 30}, {16, 30}}, color = {0, 0, 0}));
      connect(src2.terminal, line2.terminalA) annotation (
        Line(points = {{-26, -12}, {-4, -12}}, color = {0, 0, 0}));
      connect(line2.terminalB, line1.terminalB) annotation (
        Line(points = {{16, -12}, {26, -12}, {26, 30}, {16, 30}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-36, -30}, {38, -74}}, fontSize = 12, textString = "current in the load (15.08 A) is circulating
in the line 1 (2/3) and the line 2 (1/3)
voltage drop is 17 V"), Text(lineColor = {28, 108, 200}, extent = {{4, 36}, {174, 12}}, fontSize = 12, textString = "P=500 kW and Q=150 kvar")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end TwoSourcesTwoLinesOneLoad;

    model OneEmergencySourceTwoLinesOneLoad
      extends Icons.myExample;
      Components.myEmergencySource pv(Smax = 6000, UNom = 400) annotation (
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
        Line(points = {{6, -12}, {18, -12}, {18, -12.03}, {17.99, -12.03}}, color = {0, 0, 0}));
      connect(line1.terminalA, line2.terminalA) annotation (
        Line(points = {{-14, 14}, {-32, 14}, {-32, -12}, {-14, -12}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-6, 54}, {70, 32}}, textString = "P=5 kW and Q=0.1 kvar for the load
    Pmax=6 kVA for the emergency source", fontSize = 12), Text(origin = {-19, 71}, lineColor = {28, 108, 200}, extent = {{-185, -61}, {219, -165}}, textString = "the load is correctly supplied by the emergency source without ramping up
    and the voltage in the load is correct (380 V)", fontSize = 12)}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneEmergencySourceTwoLinesOneLoad;

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
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-36, -16}, {38, -60}}, fontSize = 12, textString = "upstream voltage is 10 kV
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
        Diagram(graphics = {Text(extent = {{-36, -16}, {38, -60}}, lineColor = {28, 108, 200}, fontSize = 12, textString = "source voltage is 63 kV
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
        Diagram(graphics = {Text(extent = {{-36, -20}, {38, -64}}, lineColor = {28, 108, 200}, fontSize = 12, textString = "source voltage is 63 kV
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
        Diagram(graphics = {Text(origin = {-29.513, 13}, lineColor = {28, 108, 200}, extent = {{-84.5405, -27}, {99.4603, -81}}, textString = "upstream voltage is 20 kV
downstream voltage is 10 kV
current in load is 0.289 A
current in source is 0.144 A
voltage drop at the port B of the transformer is 1.25 V", fontSize = 12), Text(lineColor = {28, 108, 200}, extent = {{-14, 52}, {144, 32}}, textString = "P=5 kW and Q=0 kvar", fontSize = 12)}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceOneTransfoOneLoad;

    model OneSourceThreeLinesThreeLoads
      extends Icons.myExample;
      Components.myLoad load1(UNom = 10000, P = 5000) annotation (
        Placement(visible = true, transformation(origin = {22, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line1(UNom = 10000, Imax = 50, R = 10) annotation (
        Placement(visible = true, transformation(origin = {-6, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load2(UNom = 10000, P = 5000) annotation (
        Placement(visible = true, transformation(origin = {22, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line3(UNom = 10000, Imax = 50, R = 10) annotation (
        Placement(visible = true, transformation(origin = {-6, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load3(UNom = 10000, P = 10000) annotation (
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
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-4, 62}, {176, 38}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-144, -26}, {148, -72}}, fontSize = 12, textString = "source voltage is 10 kV
current in line1 and line2 is 0.289 A
current in line3 is doubled (0.578 A)
voltage drop in line2 and line3 is 5 V
voltage drop in line3 is doubled (10 V)
current in the source 1.156 A"), Text(lineColor = {28, 108, 200}, extent = {{-2, 16}, {178, -8}}, fontSize = 12, textString = "P=10 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-4, 38}, {176, 14}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceThreeLinesThreeLoads;

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
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-20, 62}, {152, 28}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-36, -18}, {38, -62}}, fontSize = 12, textString = "source voltage is 20 kV
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
        Placement(transformation(extent = {{6, 0}, {26, 20}})));
      Components.myLoad load(UNom = 10000, P = 4000, Q = 3000) annotation (
        Placement(transformation(extent = {{32, 0}, {52, 20}})));
    equation
      connect(src.terminal, line1.terminalA) annotation (
        Line(points = {{-52, 10}, {-32, 10}}, color = {0, 0, 0}));
      connect(line1.terminalB, line2.terminalA) annotation (
        Line(points = {{-12, 10}, {6, 10}}, color = {0, 0, 0}));
      connect(line2.terminalB, load.terminal) annotation (
        Line(points = {{26, 10}, {40, 10}, {40, 9.97}, {41.99, 9.97}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-30, 62}, {168, 34}}, fontSize = 12, textString = "P=4 kW and Q=3 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-36, -8}, {38, -52}}, fontSize = 12, textString = "source voltage is 10 kV
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
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{2, 56}, {176, 34}}, fontSize = 12, textString = "P=50 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-40, -10}, {34, -54}}, fontSize = 12, textString = "as the load is strong
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
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{50, 58}, {126, 36}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-24, -6}, {50, -50}}, fontSize = 12, textString = "with almost perfect transformers
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
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-4, 62}, {168, 40}}, textString = "P=5 kW and Q=0 kvar", fontSize = 12), Text(origin = {-15.7297, 7.27273}, lineColor = {28, 108, 200}, extent = {{-94.2703, -5.27273}, {123.73, -63.2727}}, fontSize = 12, textString = "current is 0.047 A in the source
and 7.46 A in the load
voltage in the load is 387 V")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceOneTransfoOneLineOneTransfoOneLineOneLoad;

    model OneSourceOneTransfoOneLineOneTransfoOneLineOneLoadWithSensors
      extends Icons.myExample;
      Components.myTransformer tra2(UNomA = 20000, UNomB = 400, SNom = 0.25, R = 1e-6, X = 1e-6) annotation (
        Placement(visible = true, transformation(origin = {20, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line2(UNom = 400, Imax = 60, R = 1, X = 2.657870e-02) annotation (
        Placement(visible = true, transformation(origin = {52, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLoad load(UNom = 400, P = 5000) annotation (
        Placement(visible = true, transformation(origin = {110, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myLine line1(UNom = 20000, Imax = 60, R = 1, X = 125e-4) annotation (
        Placement(visible = true, transformation(origin = {-14, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.myTransformer tra1(UNomA = 63000, UNomB = 20000, SNom = 70, R = 0.2, X = 1e-6) annotation (
        Placement(visible = true, transformation(origin = {-52, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.mySource src(UNom = 63000) annotation (
        Placement(visible = true, transformation(origin = {-114, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Sensors.WattMeter sensor1 annotation (
        Placement(visible = true, transformation(origin = {-86, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Sensors.WattMeter sensor2 annotation (
        Placement(visible = true, transformation(origin = {84, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(line1.terminalB, tra2.terminalA) annotation (
        Line(points = {{-4, 22}, {10, 22}}));
      connect(tra2.terminalB, line2.terminalA) annotation (
        Line(points = {{30, 22}, {42, 22}}));
      connect(tra1.terminalB, line1.terminalA) annotation (
        Line(points = {{-42, 22}, {-24, 22}}, color = {0, 0, 0}));
      connect(tra1.terminalA, sensor1.terminalB) annotation (
        Line(points = {{-62, 22}, {-76, 22}}, color = {0, 0, 0}));
      connect(src.terminal, sensor1.terminalA) annotation (
        Line(points = {{-114, 22}, {-96, 22}}, color = {0, 0, 0}));
      connect(load.terminal, sensor2.terminalB) annotation (
        Line(points = {{109.96, 22.02}, {94, 22}}, color = {0, 0, 0}));
      connect(line2.terminalB, sensor2.terminalA) annotation (
        Line(points = {{62, 22}, {74, 22}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-4, 62}, {168, 40}}, textString = "P=5 kW and Q=0 kvar", fontSize = 12), Text(origin = {-15.7297, 7.2727}, lineColor = {28, 108, 200}, extent = {{-94.2703, -5.27273}, {123.73, -63.2727}}, fontSize = 12, textString = "current is 0.047 A in the source
and 7.46 A in the load
voltage in the load is 387 V")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceOneTransfoOneLineOneTransfoOneLineOneLoadWithSensors;

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
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-28, 58}, {174, 34}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-38, -16}, {36, -60}}, fontSize = 12, textString = "same results as previous model
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
        Line(points = {{85, 20}, {92, 20}, {92, 19.97}, {99.99, 19.97}}, color = {0, 0, 0}));
      connect(tra1.terminalB, line1.terminalA) annotation (
        Line(points = {{-96, 20}, {-40, 20}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-28, 58}, {174, 34}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-38, -16}, {36, -60}}, fontSize = 12, textString = "same results as previous model
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
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-28, 58}, {174, 34}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-38, -16}, {36, -60}}, fontSize = 12, textString = "same results as previous model
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
        Diagram(coordinateSystem(preserveAspectRatio = false), graphics = {Text(lineColor = {28, 108, 200}, extent = {{28, 46}, {104, 24}}, fontSize = 12, textString = "about 8 Mvar"), Text(lineColor = {28, 108, 200}, extent = {{-28, -24}, {174, -46}}, fontSize = 12, textString = "P=4 MW and Q=3 Mvar"), Text(lineColor = {28, 108, 200}, extent = {{-148, -50}, {148, -84}}, fontSize = 12, textString = "voltage after the line is increased
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
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-144, -24}, {152, -58}}, fontSize = 12, textString = "current  is 57.7 A in line
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
        Diagram(coordinateSystem(initialScale = 0.1), graphics = {Text(lineColor = {28, 108, 200}, extent = {{-142, -48}, {154, -82}}, fontSize = 12, textString = "current in all lines is 57.7 A as line parameters are identical
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
      connect(sensor1.Pmes, sensor3.PA) annotation (
        Line(points = {{-41, 37}, {-41, 64}, {-11.6, 64}}, color = {0, 0, 127}));
      connect(sensor1.Qmes, sensor3.QA) annotation (
        Line(points = {{-34, 37}, {-34, 58}, {-11.6, 58}}, color = {0, 0, 127}));
      connect(sensor1.Smes, sensor3.SA) annotation (
        Line(points = {{-27, 37}, {-27, 52}, {-11.6, 52}}, color = {0, 0, 127}));
      connect(sensor2.Pmes, sensor3.PB) annotation (
        Line(points = {{25, 37}, {25, 64}, {11.6, 64}}, color = {0, 0, 127}));
      connect(sensor2.Qmes, sensor3.QB) annotation (
        Line(points = {{32, 37}, {32, 58}, {11.8, 58}}, color = {0, 0, 127}));
      connect(sensor2.Smes, sensor3.SB) annotation (
        Line(points = {{39, 37}, {39, 52}, {11.6, 52}}, color = {0, 0, 127}));
      annotation (
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-150, -14}, {146, -48}}, fontSize = 10, textString = "the two wattmeters measure
active, reactive and apparent powers at port A and port B of the line
sensor3 calculates the drop
in active, reactive and apparent powers in the line")}),
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
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Text(lineColor = {28, 108, 200}, extent = {{-148, -16}, {144, -62}}, fontSize = 12, textString = "a fault appears at 0.4 s and is eliminated in 200 ms
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

    model OneSourceTwoLinesWithFaultOneLoad
      extends Icons.myExample;
      Components.mySource src(UNom = 10000) annotation (
        Placement(transformation(extent = {{-62, 0}, {-42, 20}})));
      Components.myLine line1(UNom = 10000, R = 10, X = 2.657870e-02, Imax = 80) annotation (
        Placement(transformation(extent = {{12, 0}, {32, 20}})));
      Components.myLineWithFault line2(UNom = 10000, R = 10, X = 2.657870e-02, Imax = 80, faultLocationPu = 0.7, RFault = 2, startTime = 0.4, stopTime = 0.6) annotation (
        Placement(transformation(extent = {{-32, 0}, {-12, 20}})));
      Components.myLoad load(UNom = 10000, P = 4000, Q = 3000) annotation (
        Placement(transformation(extent = {{40, 0}, {60, 20}})));
    equation
      connect(src.terminal, line2.terminalA) annotation (
        Line(points = {{-52, 10}, {-32, 10}}, color = {0, 0, 0}));
      connect(line2.terminalB, line1.terminalA) annotation (
        Line(points = {{-12, 10}, {12, 10}}, color = {0, 0, 0}));
      connect(line1.terminalB, load.terminal) annotation (
        Line(points = {{32, 10}, {40, 10}, {40, 10.02}, {49.96, 10.02}}, color = {0, 0, 0}));
      annotation (
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-30, 62}, {168, 34}}, fontSize = 12, textString = "P=4 kW and Q=3 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-36, -18}, {38, -62}}, fontSize = 12, textString = "a fault appears at 0.4 s and is eliminated in 200 ms
its location is at 70%% line length from port A
source voltge is 10 kV
current during default is above Imax for line2")}, coordinateSystem(initialScale = 0.1)),
        experiment(StopTime = 1));
    end OneSourceTwoLinesWithFaultOneLoad;

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
        Line(points = {{75.99, 73.97}, {66, 73.97}, {66, 74}, {58, 74}}, color = {0, 0, 0}));
      connect(tra.terminalA, line1.terminalB) annotation (
        Line(points = {{2, 74}, {-12, 74}}, color = {0, 0, 0}));
      connect(src.terminal, line1.terminalA) annotation (
        Line(points = {{-48, 74}, {-32, 74}}, color = {0, 0, 0}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio = false)),
        Diagram(coordinateSystem(preserveAspectRatio = false), graphics = {Text(lineColor = {28, 108, 200}, extent = {{-66, -12}, {58, -86}}, fontSize = 12, textString = "variable P/Q are given by
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
        Diagram(coordinateSystem(preserveAspectRatio = false), graphics = {Text(lineColor = {28, 108, 200}, extent = {{-64, -2}, {60, -76}}, fontSize = 12, textString = "reactive power control done on
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
          Diagram(coordinateSystem(preserveAspectRatio = false, initialScale = 0.1), graphics = {Text(lineColor = {28, 108, 200}, extent = {{-94, 20}, {30, -54}}, textString = "voltage evolution:
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
 
tap decrease at t=310 s, t=320 s, t=330 s, and t=340 s", fontSize = 12, horizontalAlignment = TextAlignment.Left)}),
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
          Icon(coordinateSystem(grid = {0.1, 0.1}, initialScale = 0.1), graphics = {Line(origin = {6.37017, 10.33}, points = {{-51.4286, -11.4286}, {-44.9714, 8.11429}, {-40.8571, 18.9143}, {-37.2, 26.5143}, {-33.9429, 31.2}, {-30.7429, 33.7714}, {-27.5429, 34.1714}, {-24.3429, 32.3429}, {-21.0857, 28.4}, {-17.8857, 22.5143}, {-14.2286, 13.7714}, {-9.61714, 0.685714}, {0.0571429, -29.0286}, {4.17143, -40.1143}, {7.82857, -48.1143}, {11.0286, -53.2}, {14.2857, -56.2286}, {17.4857, -57.1429}, {20.6857, -55.7714}, {23.9429, -52.2857}, {27.1429, -46.8}, {30.8, -38.4}, {35.4286, -25.6}, {40, -11.4286}}, smooth = Smooth.Bezier), Text(origin = {-112.04, 0.70086}, rotation = 180, extent = {{-89.9398, 7.90086}, {179.877, -15.7991}}, lineColor = {0, 0, 0}, textStyle = {TextStyle.Bold, TextStyle.Italic}, textString = "variable")}),
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
          Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-58, -2}, {66, -76}}, fontSize = 12, textString = "comparison between fixed transformer
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
          Icon(coordinateSystem(grid = {0.1, 0.1}, initialScale = 0.1), graphics = {Text(origin = {-119.206, -1.83247}, rotation = 180, extent = {{-55.9061, 7.76753}, {111.812, -15.5324}}, lineColor = {0, 0, 0}, textString = "ramp", textStyle = {TextStyle.Bold, TextStyle.Italic}), Line(origin = {6.37017, 10.33}, points = {{-51.4286, -11.4286}, {-44.9714, 8.11429}, {-40.8571, 18.9143}, {-37.2, 26.5143}, {-33.9429, 31.2}, {-30.7429, 33.7714}, {-27.5429, 34.1714}, {-24.3429, 32.3429}, {-21.0857, 28.4}, {-17.8857, 22.5143}, {-14.2286, 13.7714}, {-9.61714, 0.685714}, {0.0571429, -29.0286}, {4.17143, -40.1143}, {7.82857, -48.1143}, {11.0286, -53.2}, {14.2857, -56.2286}, {17.4857, -57.1429}, {20.6857, -55.7714}, {23.9429, -52.2857}, {27.1429, -46.8}, {30.8, -38.4}, {35.4286, -25.6}, {40, -11.4286}}, smooth = Smooth.Bezier)}));
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
          Diagram(coordinateSystem(preserveAspectRatio = false), graphics = {Text(lineColor = {28, 108, 200}, extent = {{-60, -20}, {64, -94}}, fontSize = 12, textString = "tap changes are induced by
the variable active power of the load")}),
          experiment(StopTime = 1000));
      end WithVariableLoad;
    end VariableTransformers;

    package BreakerTests "Particular cases where the loads are permanently supplied"
      extends Icons.myExamplesPackage;

      model DistrictWithOppositeBreakers "District with two opposite breakers"
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
        Components.myBreaker brk1(UNom = 400) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {24, 30})));
        Components.myBreaker brk2(UNom = 400) annotation (
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
        connect(tra2.terminalB, line21.terminalA) annotation (
          Line(points = {{2, -48}, {32, -48}}, color = {0, 0, 0}));
        connect(line2.terminalB, tra2.terminalA) annotation (
          Line(points = {{-44, -38}, {-44, -48}, {-18, -48}}, color = {0, 0, 0}));
        connect(brk1.terminalA, line11.terminalA) annotation (
          Line(points = {{24, 40}, {24, 48}, {32, 48}}, color = {0, 0, 0}));
        connect(brk2.terminalB, line21.terminalA) annotation (
          Line(points = {{24, -40}, {24, -48}, {32, -48}}, color = {0, 0, 0}));
        connect(load.terminal, line.terminalB) annotation (
          Line(points = {{99.99, -0.03}, {88, -0.03}, {88, 0}, {78, 0}}, color = {0, 0, 0}));
        connect(load21.terminal, line21.terminalB) annotation (
          Line(points = {{99.99, -48.03}, {76, -48.03}, {76, -48}, {52, -48}}, color = {0, 0, 0}));
        connect(load22.terminal, line211.terminalB) annotation (
          Line(points = {{99.99, -80.03}, {96, -80.03}, {96, -80}, {90, -80}}, color = {0, 0, 0}));
        connect(load12.terminal, line111.terminalB) annotation (
          Line(points = {{99.99, 79.97}, {96, 79.97}, {96, 80}, {90, 80}}, color = {0, 0, 0}));
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
          Line(points = {{52, 48}, {74, 48}, {74, 47.97}, {99.99, 47.97}}, color = {0, 0, 0}));
        connect(line11.terminalB, line111.terminalA) annotation (
          Line(points = {{52, 48}, {60, 48}, {60, 80}, {70, 80}}, color = {0, 0, 0}));
        connect(n.y, brk2.BrkOpen) annotation (
          Line(points = {{16.6, -30}, {21, -30}}, color = {255, 0, 255}));
        connect(cmd.y, n.u) annotation (
          Line(points = {{-9.4, 0}, {0, 0}, {0, -30}, {2.8, -30}}, color = {255, 0, 255}));
        connect(cmd.y, brk1.BrkOpen) annotation (
          Line(points = {{-9.4, 0}, {0, 0}, {0, 30}, {21, 30}}, color = {255, 0, 255}));
        connect(brk1.terminalB, line.terminalA) annotation (
          Line(points = {{24, 20}, {24, 0}, {58, 0}}, color = {0, 0, 0}));
        connect(brk2.terminalA, line.terminalA) annotation (
          Line(points = {{24, -20}, {24, 0}, {58, 0}}, color = {0, 0, 0}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false), graphics = {Text(lineColor = {28, 108, 200}, extent = {{-160, -42}, {80, -116}}, fontSize = 12, textString = "the position of the circuit breakers are opposite
breaker positions are changing at 0.5 s
all loads are permanently supplied")}),
          experiment(StopTime = 1));
      end DistrictWithOppositeBreakers;

      model Islanding1 "One load permanently supplied"
        extends Icons.myExample;
        Components.mySource src(UNom = 20000) annotation (
          Placement(visible = true, transformation(origin = {-88, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myBreaker brk(UNom = 400) annotation (
          Placement(transformation(extent = {{-10, 20}, {10, 40}})));
        Components.myEmergencySource pv(Smax(displayUnit = "kW") = 8000, UNom(displayUnit = "V") = 400) annotation (
          Placement(visible = true, transformation(origin = {68, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load1(P(displayUnit = "W") = 5000, Q(displayUnit = "var") = 100, UNom(displayUnit = "V") = 400) annotation (
          Placement(visible = true, transformation(origin = {62, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line2(B = 0.01, G = 0.01, Imax = 50, R = 0.1, UNom(displayUnit = "V") = 400) annotation (
          Placement(visible = true, transformation(origin = {44, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line3(B = 0.01, G = 0.01, Imax = 50, R = 0.1, UNom(displayUnit = "V") = 400) annotation (
          Placement(visible = true, transformation(origin = {44, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.BooleanStep cmd(startTime = 0.5, startValue = false) annotation (
          Placement(transformation(extent = {{-6, -6}, {6, 6}}, rotation = 0, origin = {-22, -4})));
        Components.myLine line1(UNom = 20000, Imax = 50, R = 0.1) annotation (
          Placement(visible = true, transformation(origin = {-62, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myTransformer tra(UNomA = 20000, UNomB = 400, R = 0.1) annotation (
          Placement(transformation(extent = {{-40, 20}, {-20, 40}})));
      equation
        connect(pv.terminal, line2.terminalB) annotation (
          Line(points = {{68, 42}, {60, 42}, {60, 42}, {54, 42}}, color = {0, 0, 0}));
        connect(line3.terminalB, load1.terminal) annotation (
          Line(points = {{54, 16}, {62, 16}, {62, 15.97}, {61.99, 15.97}}, color = {0, 0, 0}));
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
          Diagram(coordinateSystem(preserveAspectRatio = false), graphics = {Text(lineColor = {28, 108, 200}, extent = {{-116, -16}, {124, -90}}, fontSize = 12, textString = "the circuit breaker switches off at 0.5 s
consuming load is permanently supplied
by the source and/or by the PV node
after switching the power supplied by the PV node is over Pmax"), Text(lineColor = {28, 108, 200}, extent = {{36, 78}, {112, 56}}, fontSize = 12, textString = "P=5 kW and Q=0.1 kvar for the load
Pmax=8 kW for the PV node")}),
          experiment(StopTime = 1));
      end Islanding1;

      model Islanding2 "Two loads permanently supplied"
        extends Icons.myExample;
        Components.mySource src(UNom = 20000) annotation (
          Placement(visible = true, transformation(origin = {-86, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myBreaker brk(UNom = 400) annotation (
          Placement(transformation(extent = {{-8, -2}, {12, 18}})));
        Components.myEmergencySource pv(Smax(displayUnit = "kW") = 16000, UNom(displayUnit = "V") = 400) annotation (
          Placement(visible = true, transformation(origin = {86, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        PowerSysPro.Components.myLoad load1(P(displayUnit = "W") = 5000, Q = 100, UNom(displayUnit = "V") = 400) annotation (
          Placement(visible = true, transformation(origin = {80, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line2(UNom = 400, G = 0.01, B = 0.01, Imax = 50, R = 0.1) annotation (
          Placement(visible = true, transformation(origin = {50, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        PowerSysPro.Components.myLine line3(UNom = 400, G = 0.01, B = 0.01, Imax = 50, R = 0.1) annotation (
          Placement(visible = true, transformation(origin = {50, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.BooleanStep cmd(startTime = 0.5, startValue = false) annotation (
          Placement(transformation(extent = {{-6, -6}, {6, 6}}, rotation = 0, origin = {-20, -26})));
        Components.myLine line1(UNom = 20000, Imax = 50, R = 0.1) annotation (
          Placement(visible = true, transformation(origin = {-60, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myTransformer tra(UNomA = 20000, UNomB = 400, R = 0.1) annotation (
          Placement(transformation(extent = {{-38, -2}, {-18, 18}})));
        Components.myLoad load2(P(displayUnit = "W") = 5000, Q = 100, UNom(displayUnit = "V") = 400) annotation (
          Placement(visible = true, transformation(origin = {80, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line5(B = 0.01, G = 0.01, Imax = 50, R = 0.1, UNom(displayUnit = "V") = 400) annotation (
          Placement(visible = true, transformation(origin = {50, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(pv.terminal, line2.terminalB) annotation (
          Line(points = {{86, 42}, {60, 42}}, color = {0, 0, 0}));
        connect(line3.terminalB, load1.terminal) annotation (
          Line(points = {{60, 8}, {72, 8}, {72, 7.97}, {79.99, 7.97}}));
        connect(brk.terminalB, line2.terminalA) annotation (
          Line(points = {{12, 8}, {24, 8}, {24, 42}, {40, 42}}, color = {0, 0, 0}));
        connect(brk.terminalB, line3.terminalA) annotation (
          Line(points = {{12, 8}, {40, 8}}));
        connect(cmd.y, brk.BrkOpen) annotation (
          Line(points = {{-13.4, -26}, {2, -26}, {2, 5}}, color = {255, 0, 255}));
        connect(src.terminal, line1.terminalA) annotation (
          Line(points = {{-86, 8}, {-70, 8}}, color = {0, 0, 0}));
        connect(line1.terminalB, tra.terminalA) annotation (
          Line(points = {{-50, 8}, {-38, 8}}, color = {0, 0, 0}));
        connect(tra.terminalB, brk.terminalA) annotation (
          Line(points = {{-18, 8}, {-8, 8}}, color = {0, 0, 0}));
        connect(line5.terminalB, load2.terminal) annotation (
          Line(points = {{60, -26}, {72, -26}, {72, -26.03}, {79.99, -26.03}}, color = {0, 0, 0}));
        connect(brk.terminalB, line5.terminalA) annotation (
          Line(points = {{12, 8}, {24, 8}, {24, -26}, {40, -26}}, color = {0, 0, 0}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false), graphics = {Text(lineColor = {28, 108, 200}, extent = {{36, 78}, {112, 56}}, fontSize = 12, textString = "P=5 kW and Q=0.1 kvar for the loads
Pmax=16 kW for the PV node"), Text(lineColor = {28, 108, 200}, extent = {{-122, -30}, {118, -104}}, fontSize = 12, textString = "the circuit breaker switches off at 0.5 s
consuming loads are permanently supplied
by the source and/or by the PV node
the power supplied by the PV node is permanently below Pmax")}),
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
          Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{8, 42}, {186, 2}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-32, -8}, {42, -52}}, fontSize = 12, textString = "current is 7.22 A in each load
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
          Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-34, -14}, {40, -58}}, fontSize = 12, textString = "same results as previous model
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
          Diagram(coordinateSystem(initialScale = 0.1), graphics = {Text(lineColor = {28, 108, 200}, extent = {{-124, -42}, {116, -86}}, fontSize = 12, textString = "first model to export as an FMU is
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
          Diagram(coordinateSystem(initialScale = 0.1), graphics = {Text(lineColor = {28, 108, 200}, extent = {{14, 38}, {248, 4}}, fontSize = 12, textString = "P=5 kW and Q=0 kvar"), Text(lineColor = {28, 108, 200}, extent = {{-120, -32}, {120, -76}}, fontSize = 12, textString = "second model to export as an FMU is
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
        Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-140, 36}, {142, -22}}, fontSize = 12, textString = "here is a small MV LV academic network
the same model is then cut in 2 parts to get FMUs")}));
    end ExampleWithFMUs;

    package MediumNetworks "Medium MV-LV networks"
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
          Line(points = {{60, 38}, {70, 38}, {70, 73.97}, {77.99, 73.97}}));
        connect(line4.terminalB, load3.terminal) annotation (
          Line(points = {{60, -6}, {70, -6}, {70, -6.03}, {79.99, -6.03}}));
        connect(tra1.terminalB, line1.terminalA) annotation (
          Line(points = {{-52, 16}, {-40, 16}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line2.terminalA) annotation (
          Line(points = {{10, 16}, {24, 16}, {24, 38}, {40, 38}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line4.terminalA) annotation (
          Line(points = {{10, 16}, {24, 16}, {24, -6}, {40, -6}}, color = {0, 0, 0}));
        connect(load2.terminal, line3.terminalB) annotation (
          Line(points = {{79.99, 15.97}, {70, 15.97}, {70, 16}, {60, 16}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line3.terminalA) annotation (
          Line(points = {{10, 16}, {40, 16}}, color = {0, 0, 0}));
        connect(line2.terminalB, line5.terminalA) annotation (
          Line(points = {{60, 38}, {78, 38}}, color = {0, 0, 0}));
        connect(line5.terminalB, load4.terminal) annotation (
          Line(points = {{98, 38}, {102, 38}, {102, 37.97}, {109.99, 37.97}}, color = {0, 0, 0}));
        connect(src.terminal, tra1.terminalA) annotation (
          Line(points = {{-88, 16}, {-72, 16}, {-72, 16}}, color = {0, 0, 0}));
        annotation (
          Diagram(graphics = {Text(origin = {-8, -4},lineColor = {28, 108, 200}, extent = {{-14, 96}, {62, 74}}, textString = "All residential loads
36 kVA (Q=0.4*P)", textStyle = {TextStyle.Bold}), Text(lineColor = {28, 108, 200}, extent = {{-110, 42}, {-66, 28}}, textString = "RTE", fontSize = 10, textStyle = {TextStyle.Bold}), Text(origin = {9, -12},lineColor = {28, 108, 200}, extent = {{-48, -16}, {37, -48}}, textString = "low voltage is detected in 
load1 and load4")}, coordinateSystem(initialScale = 0.1)),
          experiment(StopTime = 1));
      end Network;

      model DoubleNetwork "Double Distribution network"
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
          Line(points = {{60, 60}, {70, 60}, {70, 95.97}, {77.99, 95.97}}));
        connect(line4.terminalB, load3.terminal) annotation (
          Line(points = {{60, 16}, {70, 16}, {70, 15.97}, {79.99, 15.97}}));
        connect(src.terminal, tra1.terminalA) annotation (
          Line(points = {{-92, 38}, {-72, 38}}, color = {0, 0, 0}));
        connect(tra1.terminalB, line1.terminalA) annotation (
          Line(points = {{-52, 38}, {-40, 38}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line2.terminalA) annotation (
          Line(points = {{10, 38}, {24, 38}, {24, 60}, {40, 60}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line4.terminalA) annotation (
          Line(points = {{10, 38}, {24, 38}, {24, 16}, {40, 16}}, color = {0, 0, 0}));
        connect(load2.terminal, line3.terminalB) annotation (
          Line(points = {{79.99, 37.97}, {70, 37.97}, {70, 38}, {60, 38}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line3.terminalA) annotation (
          Line(points = {{10, 38}, {40, 38}}, color = {0, 0, 0}));
        connect(line2.terminalB, line5.terminalA) annotation (
          Line(points = {{60, 60}, {78, 60}}, color = {0, 0, 0}));
        connect(line5.terminalB, load4.terminal) annotation (
          Line(points = {{98, 60}, {102, 60}, {102, 59.97}, {109.99, 59.97}}, color = {0, 0, 0}));
        connect(line6.terminalB, load5.terminal) annotation (
          Line(points = {{62, -40}, {72, -40}, {72, -4.03}, {79.99, -4.03}}));
        connect(line7.terminalB, load7.terminal) annotation (
          Line(points = {{62, -84}, {72, -84}, {72, -84.03}, {81.99, -84.03}}));
        connect(load6.terminal, line8.terminalB) annotation (
          Line(points = {{81.99, -62.03}, {72, -62.03}, {72, -62}, {62, -62}}, color = {0, 0, 0}));
        connect(line6.terminalB, line9.terminalA) annotation (
          Line(points = {{62, -40}, {80, -40}}, color = {0, 0, 0}));
        connect(line9.terminalB, load8.terminal) annotation (
          Line(points = {{100, -40}, {104, -40}, {104, -40.03}, {111.99, -40.03}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line6.terminalA) annotation (
          Line(points = {{10, 38}, {24, 38}, {24, -40}, {42, -40}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line8.terminalA) annotation (
          Line(points = {{10, 38}, {24, 38}, {24, -62}, {42, -62}}, color = {0, 0, 0}));
        connect(tra2.terminalB, line7.terminalA) annotation (
          Line(points = {{10, 38}, {24, 38}, {24, -84}, {42, -84}}, color = {0, 0, 0}));
        annotation (
          Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-114, 64}, {-70, 50}}, textString = "RTE", fontSize = 10, textStyle = {TextStyle.Bold}), Text(origin = {63, 6},lineColor = {28, 108, 200}, extent = {{-191, 6}, {-47, -64}}, textString = "same network with two LV feeders
low voltage is detected in 
load1, load4, load5 and load8
nominal power exceed detected in tra2"), Text(origin = {-8, -2},lineColor = {28, 108, 200}, extent = {{-24, 98}, {52, 76}}, textString = "All residential loads
36 kVA (Q=0.4*P)", textStyle = {TextStyle.Bold})}, coordinateSystem(initialScale = 0.1)),
          experiment(StopTime = 1));
      end DoubleNetwork;
    end MediumNetworks;

    package StructuredNetwork "Structured network using subnetwork components"
      extends Icons.myExamplesPackage;

      model DoubleNetworkWithLoads "Double Distribution network"
        extends Icons.myExample;
        Source src annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-32, 30})));
        FeederWithLoads feeder2 annotation (
          Placement(transformation(extent = {{24, 4}, {44, 24}})));
        FeederWithLoads feeder1 annotation (
          Placement(transformation(extent = {{24, 34}, {44, 54}})));
      equation
        connect(src.terminal, feeder1.terminal) annotation (
          Line(points = {{-22.2, 30}, {0, 30}, {0, 44}, {24.2, 44}}, color = {0, 0, 0}));
        connect(src.terminal, feeder2.terminal) annotation (
          Line(points = {{-22.2, 30}, {0, 30}, {0, 14}, {24.2, 14}}, color = {0, 0, 0}));
        annotation (
          Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-96, -12}, {102, -60}}, textString = "another way to model MediumNetworks.DoubleNetwork
using subnetwork components with loads")}, coordinateSystem(initialScale = 0.1)),
          experiment(StopTime = 1));
      end DoubleNetworkWithLoads;

      model DoubleNetworkWithVariableLoads "Double Distribution network"
        extends Icons.myExample;
        Source src annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-32, 30})));
        FeederWithVariableLoads feeder2 annotation (
          Placement(transformation(extent = {{24, 4}, {44, 24}})));
        FeederWithVariableLoads feeder1 annotation (
          Placement(transformation(extent = {{24, 34}, {44, 54}})));
      equation
        connect(src.terminal, feeder1.terminal) annotation (
          Line(points = {{-22.2, 30}, {0, 30}, {0, 44}, {24.2, 44}}, color = {0, 0, 0}));
        connect(src.terminal, feeder2.terminal) annotation (
          Line(points = {{-22.2, 30}, {0, 30}, {0, 14}, {24.2, 14}}, color = {0, 0, 0}));
        annotation (
          Diagram(graphics = {Text(lineColor = {28, 108, 200}, extent = {{-96, -12}, {102, -60}}, textString = "another way to model MediumNetworks.DoubleNetwork
using subnetwork components with variable loads")}, coordinateSystem(initialScale = 0.1)),
          experiment(StopTime = 1));
      end DoubleNetworkWithVariableLoads;

      model DoubleNetworkWithTwoOppositeBreakers
        "Double Distribution network with two opposite breakers"
        extends Icons.myExample;
        Source src1 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={-42,44})));
        Source src2 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={-42,0})));
        FeederWithLoads feeder2
          annotation (Placement(transformation(extent={{24,-10},{44,10}})));
        FeederWithLoads feeder1
          annotation (Placement(transformation(extent={{24,34},{44,54}})));
        Components.myBreaker brk1(UNom = 400) annotation (
          Placement(transformation(extent = {{-22, 34}, {-2, 54}})));
        Components.myBreaker brk2(UNom = 400) annotation (
          Placement(transformation(extent = {{-22, -10}, {-2, 10}})));
        Modelica.Blocks.Sources.BooleanPulse cmd(width = 50, period = 1, startTime = 0.25) annotation (
          Placement(transformation(extent = {{-102, 14}, {-88, 28}})));
        Modelica.Blocks.Logical.Not n annotation (
          Placement(transformation(extent = {{-30, -26}, {-16, -12}})));
      equation
        // As breakers are opposite, feeders are permanently supplied so the breakers must simply cut the current, not the voltage
        // Managing the global variables
        //        feeder1.Supplied = true;
        //      feeder2.Supplied = true;
        connect(src1.terminal, brk1.terminalA) annotation (
          Line(points = {{-32.2, 44}, {-22, 44}}, color = {0, 0, 0}));
        connect(brk1.terminalB, feeder1.terminal)
          annotation (Line(points={{-2,44},{24.2,44}}, color={0,0,0}));
        connect(brk1.terminalB, feeder2.terminal) annotation (Line(points={{-2,
                44},{8,44},{8,0},{24.2,0}}, color={0,0,0}));
        connect(src2.terminal, brk2.terminalA) annotation (
          Line(points = {{-32.2, -1.72085e-15}, {-22, 0}}, color = {0, 0, 0}));
        connect(brk2.terminalB, feeder2.terminal)
          annotation (Line(points={{-2,0},{24.2,0}}, color={0,0,0}));
        connect(brk1.terminalB, brk2.terminalB) annotation (
          Line(points = {{-2, 44}, {8, 44}, {8, 0}, {-2, 0}}, color = {0, 0, 0}));
        connect(brk2.terminalB, feeder1.terminal) annotation (Line(points={{-2,
                0},{8,0},{8,44},{24.2,44}}, color={0,0,0}));
        connect(cmd.y, brk1.BrkOpen) annotation (
          Line(points = {{-87.3, 21}, {-12, 21}, {-12, 41}}, color = {255, 0, 255}));
        connect(n.y, brk2.BrkOpen) annotation (
          Line(points = {{-15.3, -19}, {-12, -19}, {-12, -3}}, color = {255, 0, 255}));
        connect(n.u, brk1.BrkOpen) annotation (
          Line(points = {{-31.4, -19}, {-80, -19}, {-80, 21}, {-12, 21}, {-12, 41}}, color = {255, 0, 255}));
        annotation (
          Diagram(coordinateSystem(initialScale = 0.1), graphics={Text(origin = {1, -10},lineColor = {28, 108, 200}, extent = {{-149, -30}, {151, -70}}, textString = "the two feeders are permanently supplied
by one source or the other
as the breakers are opposite
(they can simply cut the current)")}),
          experiment(StopTime = 1));
      end DoubleNetworkWithTwoOppositeBreakers;

      model Source "Upstream part of the networks"
        extends Components.PartialModels.mySubNetworkOnePort;
        Components.myTransformer tra1(UNomA = 63000, UNomB = 20000, SNom = 36000, R = 1e-9, X = 1e-9) annotation (
          Placement(visible = true, transformation(origin = {-28, 66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line1(UNom = 20000, Imax = 160, R = 0.9, X = 0.3) annotation (
          Placement(visible = true, transformation(origin = {4, 66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myTransformer tra2(UNomA = 20000, UNomB = 400, SNom = 250, R = 1e-9, X = 1e-9) annotation (
          Placement(visible = true, transformation(origin = {34, 66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.mySource src(UNom = 63000) annotation (
          Placement(visible = true, transformation(origin = {-54, 66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(line1.terminalB, tra2.terminalA) annotation (
          Line(points = {{14, 66}, {24, 66}}));
        connect(tra1.terminalB, line1.terminalA) annotation (
          Line(points = {{-18, 66}, {-6, 66}}, color = {0, 0, 0}));
        connect(src.terminal, tra1.terminalA) annotation (
          Line(points = {{-54, 66}, {-38, 66}}, color = {0, 0, 0}));
        connect(tra2.terminalB, terminal) annotation (
          Line(points = {{44, 66}, {60, 66}, {60, 0}, {-98, 0}}, color = {0, 0, 0}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false), graphics={  Text(lineColor = {28, 108, 200}, extent = {{-76, 92}, {-32, 78}}, textStyle = {TextStyle.Bold}, fontSize = 10, textString = "RTE")}));
      end Source;

      model FeederWithLoads "Downstream part of the networks"
        extends Components.PartialModels.mySubNetworkOnePort;
        Components.myLoad load1(P = 36000 / sqrt(1.16), UNom = 400, Q = 0.4 * 36000 / sqrt(1.16)) annotation (
          Placement(visible = true, transformation(origin = {64, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line2(UNom = 400, Imax = 140, R = 0.2, X = 0.2) annotation (
          Placement(visible = true, transformation(origin = {36, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load2(P = 36000 / sqrt(1.16), UNom = 400, Q = 0.4 * 36000 / sqrt(1.16)) annotation (
          Placement(visible = true, transformation(origin = {66, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line4(UNom = 400, Imax = 140, R = 0.2, X = 0.2) annotation (
          Placement(visible = true, transformation(origin = {36, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load3(P = 36000 / sqrt(1.16), UNom = 400, Q = 0.4 * 36000 / sqrt(1.16)) annotation (
          Placement(visible = true, transformation(origin = {66, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line3(UNom = 400, Imax = 140, R = 0.2, X = 0.2) annotation (
          Placement(visible = true, transformation(origin = {36, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line5(UNom = 400, Imax = 140, R = 0.2, X = 0.2) annotation (
          Placement(visible = true, transformation(origin = {74, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLoad load4(P = 36000 / sqrt(1.16), UNom = 400, Q = 0.4 * 36000 / sqrt(1.16)) annotation (
          Placement(visible = true, transformation(origin = {96, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(line2.terminalB, load1.terminal) annotation (
          Line(points = {{46, 22}, {56, 22}, {56, 57.97}, {63.99, 57.97}}));
        connect(line4.terminalB, load3.terminal) annotation (
          Line(points = {{46, -22}, {56, -22}, {56, -22.03}, {65.99, -22.03}}));
        connect(load2.terminal, line3.terminalB) annotation (
          Line(points = {{65.99, -0.03}, {56, -0.03}, {56, 0}, {46, 0}}, color = {0, 0, 0}));
        connect(line2.terminalB, line5.terminalA) annotation (
          Line(points = {{46, 22}, {64, 22}}, color = {0, 0, 0}));
        connect(line5.terminalB, load4.terminal) annotation (
          Line(points = {{84, 22}, {88, 22}, {88, 21.97}, {95.99, 21.97}}, color = {0, 0, 0}));
        connect(line3.terminalA, terminal) annotation (
          Line(points = {{26, 0}, {-36, 0}, {-36, 0}, {-98, 0}}, color = {0, 0, 0}));
        connect(line4.terminalA, terminal) annotation (
          Line(points = {{26, -22}, {0, -22}, {0, 0}, {-98, 0}}, color = {0, 0, 0}));
        connect(line2.terminalA, terminal) annotation (
          Line(points = {{26, 22}, {0, 22}, {0, 0}, {-98, 0}}, color = {0, 0, 0}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false), graphics = {Text(origin = {-14, -12},lineColor = {28, 108, 200}, extent = {{-28, 80}, {48, 58}}, textString = "All residential loads
36 kVA (Q=0.4*P)", textStyle = {TextStyle.Bold})}));
      end FeederWithLoads;

      model FeederWithVariableLoads "Downstream part of the networks"
        extends Components.PartialModels.mySubNetworkOnePort;
        Components.myVariableLoad load1(UNom = 400, switchToImpedanceMode = false) annotation (
          Placement(visible = true, transformation(origin = {64, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line2(UNom = 400, Imax = 140, R = 0.2, X = 0.2) annotation (
          Placement(visible = true, transformation(origin = {36, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myVariableLoad load2(UNom = 400, switchToImpedanceMode = false) annotation (
          Placement(visible = true, transformation(origin = {66, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line4(UNom = 400, Imax = 140, R = 0.2, X = 0.2) annotation (
          Placement(visible = true, transformation(origin = {36, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myVariableLoad load3(UNom = 400, switchToImpedanceMode = false) annotation (
          Placement(visible = true, transformation(origin = {66, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line3(UNom = 400, Imax = 140, R = 0.2, X = 0.2) annotation (
          Placement(visible = true, transformation(origin = {36, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myLine line5(UNom = 400, Imax = 140, R = 0.2, X = 0.2) annotation (
          Placement(visible = true, transformation(origin = {74, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Components.myVariableLoad load4(UNom = 400, switchToImpedanceMode = false) annotation (
          Placement(visible = true, transformation(origin = {96, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.RealExpression P(y = 36000 / sqrt(1.16)) annotation (
          Placement(transformation(extent = {{-122, -64}, {-62, -40}})));
        Modelica.Blocks.Sources.RealExpression Q(y = 0.4 * 36000 / sqrt(1.16)) annotation (
          Placement(transformation(extent = {{-122, -86}, {-62, -66}})));
      equation
        connect(line2.terminalB, load1.terminal) annotation (
          Line(points = {{46, 22}, {56, 22}, {56, 57.97}, {63.99, 57.97}}));
        connect(line4.terminalB, load3.terminal) annotation (
          Line(points = {{46, -22}, {56, -22}, {56, -22.03}, {65.99, -22.03}}));
        connect(load2.terminal, line3.terminalB) annotation (
          Line(points = {{65.99, -0.03}, {56, -0.03}, {56, 0}, {46, 0}}, color = {0, 0, 0}));
        connect(line2.terminalB, line5.terminalA) annotation (
          Line(points = {{46, 22}, {64, 22}}, color = {0, 0, 0}));
        connect(line5.terminalB, load4.terminal) annotation (
          Line(points = {{84, 22}, {88, 22}, {88, 21.97}, {95.99, 21.97}}, color = {0, 0, 0}));
        connect(line3.terminalA, terminal) annotation (
          Line(points = {{26, 0}, {-36, 0}, {-36, 0}, {-98, 0}}, color = {0, 0, 0}));
        connect(line4.terminalA, terminal) annotation (
          Line(points = {{26, -22}, {0, -22}, {0, 0}, {-98, 0}}, color = {0, 0, 0}));
        connect(line2.terminalA, terminal) annotation (
          Line(points = {{26, 22}, {0, 22}, {0, 0}, {-98, 0}}, color = {0, 0, 0}));
        connect(P.y, load3.PInput) annotation (
          Line(points = {{-59, -52}, {0, -52}, {0, -48}, {63.46, -48}, {63.46, -30.45}}, color = {0, 0, 127}));
        connect(P.y, load2.PInput) annotation (
          Line(points = {{-59, -52}, {22, -52}, {22, -46}, {94, -46}, {94, -12}, {63.46, -12}, {63.46, -8.45}}, color = {0, 0, 127}));
        connect(P.y, load1.PInput) annotation (
          Line(points = {{-59, -52}, {52, -52}, {52, -48}, {164, -48}, {164, 48}, {90, 48}, {90, 36}, {64, 36}, {64, 49.55}, {61.46, 49.55}}, color = {0, 0, 127}));
        connect(P.y, load4.PInput) annotation (
          Line(points = {{-59, -52}, {58, -52}, {58, -62}, {148, -62}, {148, 8}, {93.46, 8}, {93.46, 13.55}}, color = {0, 0, 127}));
        connect(Q.y, load3.QInput) annotation (
          Line(points = {{-59, -76}, {8, -76}, {8, -78}, {68.565, -78}, {68.565, -30.435}}, color = {0, 0, 127}));
        connect(Q.y, load2.QInput) annotation (
          Line(points = {{-59, -76}, {34, -76}, {34, -68}, {118, -68}, {118, -8.435}, {68.565, -8.435}}, color = {0, 0, 127}));
        connect(Q.y, load1.QInput) annotation (
          Line(points = {{-59, -76}, {184, -76}, {184, 40}, {72, 40}, {72, 49.565}, {66.565, 49.565}}, color = {0, 0, 127}));
        connect(Q.y, load4.QInput) annotation (
          Line(points = {{-59, -76}, {54, -76}, {54, -72}, {158, -72}, {158, -2}, {98.565, -2}, {98.565, 13.565}}, color = {0, 0, 127}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false), graphics={Text(lineColor = {28, 108, 200}, extent = {{-28, 80}, {48, 58}}, textString = "All residential loads
36 kVA (Q=0.4*P)", textStyle = {TextStyle.Bold})}));
      end FeederWithVariableLoads;
    end StructuredNetwork;

    model FourVariableLoadsThreeFeeders "Realistic case with three LV feeders and four variable loads"
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
        Line(points = {{42, 40}, {48, 40}, {48, 39.97}, {51.99, 39.97}}, color = {0, 0, 0}));
      connect(LVln2.terminalB, varLoad3.terminal) annotation (
        Line(points = {{28, 0}, {32, 0}, {32, -0.03}, {51.99, -0.03}}, color = {0, 0, 0}));
      connect(LVln3.terminalB, varLoad4.terminal) annotation (
        Line(points = {{28, -40}, {32, -40}, {32, -40.03}, {49.99, -40.03}}, color = {0, 0, 0}));
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
        Line(points = {{42, 40}, {42, 79.97}, {53.99, 79.97}}, color = {0, 0, 0}));
      annotation (
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Text(origin = {24, 5},lineColor = {28, 108, 200}, extent = {{-158, 73}, {-12, 53}}, textString = "all loads are variable
simulation is done for 1-year duration")}),
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}})),
        experiment(StopTime = 31532400));
    end FourVariableLoadsThreeFeeders;

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
        Line(points = {{-32, 104}, {-28, 104}, {-28, 103.97}, {-20.01, 103.97}}, color = {0, 0, 0}));
      connect(ln703.terminalB, b61260.terminal) annotation (
        Line(points = {{-8, 80}, {-2, 80}, {-2, 79.97}, {5.99, 79.97}}, color = {0, 0, 0}));
      connect(ln676.terminalB, ln703.terminalA) annotation (
        Line(points = {{-32, 104}, {-32, 80}, {-28, 80}}, color = {0, 0, 0}));
      connect(ln713.terminalB, mvlv11.terminalA) annotation (
        Line(points = {{16, 56}, {26, 56}}, color = {0, 0, 0}));
      connect(ln771.terminalB, b61275.terminal) annotation (
        Line(points = {{80, 56}, {86, 56}, {86, 55.97}, {93.99, 55.97}}, color = {0, 0, 0}));
      connect(ln770.terminalB, b61274.terminal) annotation (
        Line(points = {{80, 34}, {93.99, 34}, {93.99, 33.97}}, color = {0, 0, 0}));
      connect(ln774.terminalB, b61273.terminal) annotation (
        Line(points = {{106, 14}, {108, 14}, {108, 13.97}, {109.99, 13.97}}, color = {0, 0, 0}));
      connect(ln769.terminalB, b61254.terminal) annotation (
        Line(points = {{78, -6}, {84, -6}, {84, -6.03}, {91.99, -6.03}}, color = {0, 0, 0}));
      connect(mvlv11.terminalB, ln769.terminalA) annotation (
        Line(points = {{46, 56}, {46, -8}, {58, -8}, {58, -6}}, color = {0, 0, 0}));
      connect(mvlv11.terminalB, ln771.terminalA) annotation (
        Line(points = {{46, 56}, {60, 56}}, color = {0, 0, 0}));
      connect(mvlv11.terminalB, ln770.terminalA) annotation (
        Line(points = {{46, 56}, {52, 56}, {52, 34}, {60, 34}}, color = {0, 0, 0}));
      connect(ln769.terminalB, ln772.terminalA) annotation (
        Line(points = {{78, -6}, {78, -32}, {82, -32}}, color = {0, 0, 0}));
      connect(ln772.terminalB, b11407.terminal) annotation (
        Line(points = {{102, -32}, {112, -32}, {112, -32.03}, {117.99, -32.03}}, color = {0, 0, 0}));
      connect(ln772.terminalB, ln773.terminalA) annotation (
        Line(points = {{102, -32}, {102, -56}, {116, -56}}, color = {0, 0, 0}));
      connect(ln773.terminalB, b11403.terminal) annotation (
        Line(points = {{136, -56}, {140, -56}, {140, -56.03}, {145.99, -56.03}}, color = {0, 0, 0}));
      connect(ln676.terminalB, ln702.terminalA) annotation (
        Line(points = {{-32, 104}, {-32, -26}}, color = {0, 0, 0}));
      connect(ln703.terminalB, ln713.terminalA) annotation (
        Line(points = {{-8, 80}, {-8, 56}, {-4, 56}}, color = {0, 0, 0}));
      connect(ln702.terminalB, b61227.terminal) annotation (
        Line(points = {{-12, -26}, {-4, -26}, {-4, -26.03}, {-0.01, -26.03}}, color = {0, 0, 0}));
      connect(ln702.terminalB, ln712.terminalA) annotation (
        Line(points = {{-12, -26}, {-8, -26}, {-8, -58}, {-4, -58}}, color = {0, 0, 0}));
      connect(ln712.terminalB, mvlv24.terminalA) annotation (
        Line(points = {{16, -58}, {26, -58}}, color = {0, 0, 0}));
      connect(mvlv24.terminalB, ln775.terminalA) annotation (
        Line(points = {{46, -58}, {54, -58}}, color = {0, 0, 0}));
      connect(ln775.terminalB, b56131.terminal) annotation (
        Line(points = {{74, -58}, {76, -58}, {76, -58.03}, {85.99, -58.03}}, color = {0, 0, 0}));
      connect(ln775.terminalB, ln776.terminalA) annotation (
        Line(points = {{74, -58}, {74, -80}, {82, -80}}, color = {0, 0, 0}));
      connect(ln776.terminalB, b87656.terminal) annotation (
        Line(points = {{102, -80}, {102, -80.03}, {109.99, -80.03}}, color = {0, 0, 0}));
      connect(ln775.terminalB, ln777.terminalA) annotation (
        Line(points = {{74, -58}, {74, -100}, {82, -100}}, color = {0, 0, 0}));
      connect(ln777.terminalB, b61106.terminal) annotation (
        Line(points = {{102, -100}, {104, -100}, {104, -100.03}, {109.99, -100.03}}, color = {0, 0, 0}));
      connect(ln712.terminalB, ln720.terminalA) annotation (
        Line(points = {{16, -58}, {16, -84}, {20, -84}}, color = {0, 0, 0}));
      connect(ln720.terminalB, b61089.terminal) annotation (
        Line(points = {{40, -84}, {44, -84}, {44, -84.03}, {47.99, -84.03}}, color = {0, 0, 0}));
      connect(ln677.terminalB, b61280.terminal) annotation (
        Line(points = {{-150, 40}, {-138, 40}, {-138, 65.97}, {-130.01, 65.97}}, color = {0, 0, 0}));
      connect(ln677.terminalB, ln715.terminalA) annotation (
        Line(points = {{-150, 40}, {-140, 40}, {-140, 36}, {-132, 36}}, color = {0, 0, 0}));
      connect(ln715.terminalB, b61256.terminal) annotation (
        Line(points = {{-112, 36}, {-106, 36}, {-106, 35.97}, {-100.01, 35.97}}, color = {0, 0, 0}));
      connect(ln718.terminalB, b11480.terminal) annotation (
        Line(points = {{-80, 12}, {-76, 12}, {-76, 11.97}, {-72.01, 11.97}}, color = {0, 0, 0}));
      connect(ln677.terminalB, ln714.terminalA) annotation (
        Line(points = {{-150, 40}, {-150, -20}, {-142, -20}}, color = {0, 0, 0}));
      connect(ln717.terminalB, b61121.terminal) annotation (
        Line(points = {{-82, -42}, {-74, -42}, {-74, -42.03}, {-66.01, -42.03}}, color = {0, 0, 0}));
      connect(ln716.terminalB, ln719.terminalA) annotation (
        Line(points = {{-80, -66}, {-80, -92}, {-70, -92}}, color = {0, 0, 0}));
      connect(ln719.terminalB, b87694.terminal) annotation (
        Line(points = {{-50, -92}, {-50, -92.03}, {-40.01, -92.03}}, color = {0, 0, 0}));
      connect(ln716.terminalB, b61232.terminal) annotation (
        Line(points = {{-80, -66}, {-72, -66}, {-72, -66.03}, {-66.01, -66.03}}, color = {0, 0, 0}));
      connect(ln714.terminalB, ln716.terminalA) annotation (
        Line(points = {{-122, -20}, {-122, -66}, {-100, -66}}, color = {0, 0, 0}));
      connect(ln714.terminalB, b11458.terminal) annotation (
        Line(points = {{-122, -20}, {-122, -20.03}, {-96.01, -20.03}}, color = {0, 0, 0}));
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
        Diagram(coordinateSystem(initialScale = 0.1), graphics = {Text(lineColor = {28, 108, 200}, extent = {{-206, -2}, {198, -66}}, fontSize = 12, textString = "The load switches to a degraded
mode at time 36 s (when voltage is too low)
and switches again to a normal mode
at time 364 s (when voltage is normal again)
The apparent power of the load is reduced
and can be compared to the required one")}),
        experiment(StopTime = 400));
    end DegradedModeForLoads;

    package UsingBatteries "Examples using batteries"
      extends Icons.myExamplesPackage;

      model ChargingBatteries "Charging mode"
        extends Icons.myExample;
        PowerSysPro.Components.mySource src1(UNom = 400) annotation (
          Placement(transformation(extent = {{-46, 50}, {-26, 70}})));
        PowerSysPro.Components.myLine line1(UNom = 400, R = 0.1) annotation (
          Placement(transformation(extent = {{-12, 50}, {8, 70}})));
        PowerSysPro.Components.myBattery bat1(Np = 1, Ns = 96, SOC_start = 0.2, UNom(displayUnit = "V") = 400) annotation (
          Placement(transformation(extent = {{24, 50}, {44, 70}})));
        PowerSysPro.Components.myBattery bat2(Np = 4, Ns = 96, SOC_start = 0.2, UNom(displayUnit = "V") = 400) annotation (
          Placement(visible = true, transformation(extent = {{24, 22}, {44, 42}}, rotation = 0)));
        Modelica.Blocks.Sources.BooleanExpression charging(y = true) annotation (
          Placement(visible = true, transformation(origin = {-4, 86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        PowerSysPro.Components.myBattery bat3(Np = 8, Ns = 96, SOC_start = 0.2, UNom(displayUnit = "V") = 400) annotation (
          Placement(visible = true, transformation(extent = {{24, -8}, {44, 12}}, rotation = 0)));
      equation
        connect(src1.terminal, line1.terminalA) annotation (
          Line(points = {{-36, 60}, {-12, 60}}, color = {0, 0, 0}));
        connect(line1.terminalB, bat1.terminal) annotation (
          Line(points = {{8, 60}, {34, 60}}, color = {0, 0, 0}));
        connect(charging.y, bat1.ChargingMode) annotation (
          Line(points = {{7, 86}, {18, 86}, {18, 63}, {24, 63}}, color = {255, 0, 255}));
        connect(charging.y, bat2.ChargingMode) annotation (
          Line(points = {{7, 86}, {18, 86}, {18, 35}, {24, 35}}, color = {255, 0, 255}));
        connect(charging.y, bat3.ChargingMode) annotation (
          Line(points={{7,86},{18,86},{18,5},{24,5}},          color = {255, 0, 255}));
      connect(line1.terminalB, bat2.terminal) annotation (
          Line(points = {{8, 60}, {8, 32}, {34, 32}}));
      connect(line1.terminalB, bat3.terminal) annotation (
          Line(points = {{8, 60}, {8, 2}, {34, 2}}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false), graphics={  Text(origin = {-8, 42}, lineColor = {28, 108, 200}, extent = {{-126, -66}, {132, -109}}, textString = "all batteries are loaded in 5 hr 24 min (19440 s)
      SOC is increasing from 0.2 to 0.8 at the same time
      the current flowing bat2 is four times greater than the current flowing bat1
      the current flowing bat3 is two times greater than the current flowing bat2"), Text(lineColor = {28, 108, 200}, extent = {{6, 74}, {158, 52}}, textString = "Ns = 96 & Np = 1", fontSize = 12), Text(origin = {2, 12}, lineColor = {28, 108, 200}, extent = {{4, 32}, {156, 10}}, textString = "Ns = 96 & Np = 4", fontSize = 12), Text(origin = {2, -18}, lineColor = {28, 108, 200}, extent = {{4, 32}, {156, 10}}, textString = "Ns = 96 & Np = 8", fontSize = 12)}),
          experiment(StopTime = 36000, __Dymola_Algorithm = "Dassl"));
      end ChargingBatteries;

      model OneBatteryOneLineOneLoad "Second discharging case"
        extends Icons.myExample;
        PowerSysPro.Components.myBattery bat(Np = 80, Ns = 96, SOC_start = 0.8, UNom(displayUnit = "V") = 400) annotation (
          Placement(visible = true, transformation(extent = {{-20, 30}, {0, 50}}, rotation = 0)));
        PowerSysPro.Components.myLoad load(Disengageable = true, P(displayUnit = "W") = 5000, UNom(displayUnit = "V") = 400) annotation (
          Placement(visible = true, transformation(extent = {{58, 30}, {78, 50}}, rotation = 0)));
        PowerSysPro.Components.myLine line(UNom = 400, R = 0.1) annotation (
          Placement(visible = true, transformation(extent = {{20, 30}, {40, 50}}, rotation = 0)));
        Modelica.Blocks.Sources.BooleanExpression charging(y = false) annotation (
          Placement(visible = true, transformation(extent = {{-74, 22}, {-54, 42}}, rotation = 0)));
      equation
        connect(
          load.terminal, line.terminalB) annotation (
          Line(points={{67.99,39.97},{54,39.97},{54,40},{40,40}}));
        connect(
          bat.terminal, line.terminalA)
          annotation (
          Line(points = {{-10, 40}, {20, 40}}));
        connect(
          charging.y, bat.ChargingMode) annotation (
          Line(points={{-53,32},{-36,32},{-36,43},{-20,43}},          color = {255, 0, 255}));
        annotation (
          experiment(StopTime = 360000, __Dymola_Algorithm = "Dassl"),
          Diagram(graphics={  Text(origin = {2, 82}, lineColor = {28, 108, 200}, extent = {{-118, -88}, {123, -142}}, textString = "battery can be used >14 hrs  (50634 s)
 (SOC is decresasing from 0.8 to 0.2)
 decrease the Np value to check the battery time duration is shorter"), Text(origin = {-28, -2}, lineColor = {28, 108, 200}, extent = {{-58, 74}, {94, 52}}, textString = "Ns = 96 & Np = 80", fontSize = 12), Text(origin = {-48, 6}, lineColor = {28, 108, 200}, extent = {{24, 28}, {222, 0}}, textString = "P=5 kW and Q=0 kvar", fontSize = 12)}));
      end OneBatteryOneLineOneLoad;

      model LoadRescuedByBattery "Using a battery in rescue mode"
        extends Icons.myExample;
        PowerSysPro.Components.mySource src(UNom = 400) annotation (
          Placement(transformation(extent = {{-108, 34}, {-88, 54}})));
        PowerSysPro.Components.myLine line1(UNom = 400, R = 0.1) annotation (
          Placement(transformation(extent = {{-36, 74}, {-16, 94}})));
        PowerSysPro.Components.myBattery bat(Np = 36, Ns = 96, SOC_start = 0.6, UNom(displayUnit = "V") = 400, efficiency = 0.9) annotation (
          Placement(transformation(extent = {{0, 74}, {20, 94}})));
        PowerSysPro.Components.myLine line3(UNom = 400, R = 0.1) annotation (
          Placement(transformation(extent = {{-42, -6}, {-22, 14}})));
        PowerSysPro.Components.myLoad load(Disengageable = true, P(displayUnit = "W") = 5000, UNom(displayUnit = "V") = 400) annotation (
          Placement(transformation(extent = {{96, 36}, {116, 56}})));
        PowerSysPro.Components.myBreaker brk2(UNom = 400) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 0, origin = {36, 84})));
        PowerSysPro.Components.myBreaker brk3(UNom = 400) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 0, origin = {30, 4})));
        Modelica.Blocks.Sources.BooleanStep cmd(startTime = 3600, startValue = true) annotation (
          Placement(transformation(extent = {{-6, -6}, {6, 6}}, rotation = 0, origin = {-20, 48})));
        Modelica.Blocks.Logical.Not n annotation (
          Placement(transformation(extent = {{-6, -6}, {6, 6}}, rotation = 0, origin = {12, 36})));
        PowerSysPro.Components.myBreaker brk1(UNom = 400) annotation (
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 0, origin = {-66, 84})));
        PowerSysPro.Components.myLine line2(UNom = 400, R = 0.1) annotation (
          Placement(transformation(extent = {{60, 74}, {80, 94}})));
      equation
        connect(line1.terminalB, bat.terminal) annotation (
          Line(points = {{-16, 84}, {10, 84}}, color = {0, 0, 0}));
        connect(bat.terminal, brk2.terminalA) annotation (
          Line(points = {{10, 84}, {26, 84}}, color = {0, 0, 0}));
        connect(line3.terminalB, brk3.terminalA) annotation (
          Line(points = {{-22, 4}, {20, 4}}, color = {0, 0, 0}));
        connect(cmd.y, brk2.BrkOpen) annotation (
          Line(points = {{-13.4, 48}, {36, 48}, {36, 81}}, color = {255, 0, 255}));
        connect(cmd.y, n.u) annotation (
          Line(points = {{-13.4, 48}, {-2, 48}, {-2, 36}, {4.8, 36}}, color = {255, 0, 255}));
        connect(brk1.terminalB, line1.terminalA) annotation (
          Line(points = {{-56, 84}, {-36, 84}}, color = {0, 0, 0}));
        connect(n.y, brk3.BrkOpen) annotation (
          Line(points = {{18.6, 36}, {46, 36}, {46, -10}, {30, -10}, {30, 1}}, color = {255, 0, 255}));
        connect(n.y, brk1.BrkOpen) annotation (
          Line(points = {{18.6, 36}, {46, 36}, {46, 22}, {-66, 22}, {-66, 81}}, color = {255, 0, 255}));
        connect(brk1.terminalA, src.terminal) annotation (
          Line(points = {{-76, 84}, {-86, 84}, {-86, 44}, {-98, 44}}, color = {0, 0, 0}));
        connect(src.terminal, line3.terminalA) annotation (
          Line(points = {{-98, 44}, {-86, 44}, {-86, 4}, {-42, 4}}, color = {0, 0, 0}));
        connect(brk2.terminalB, line2.terminalA) annotation (
          Line(points = {{46, 84}, {60, 84}}, color = {0, 0, 0}));
        connect(line2.terminalB, load.terminal) annotation (
          Line(points = {{80, 84}, {92, 84}, {92, 45.97}, {105.99, 45.97}}, color = {0, 0, 0}));
        connect(brk3.terminalB, load.terminal) annotation (
          Line(points = {{40, 4}, {92, 4}, {92, 45.97}, {105.99, 45.97}}, color = {0, 0, 0}));
        connect(cmd.y, bat.ChargingMode) annotation (
          Line(points = {{-13.4, 48}, {-6, 48}, {-6, 87}, {0, 87}}, color = {255, 0, 255}));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false), graphics={Text(origin = {-13, 115}, lineColor = {28, 108, 200}, extent = {{-265, -135}, {295, -197}}, textString = "breakers 1 & 3 have the same position
breaker 2 has an opposite position 
switching of positions occurs at 3600 s
during first hour, the battery is loaded (SOC is increasing from 0.6 to 0.71)
then battery is supplying the load for the remaining 5 hours
the load is permanently supplied without any cutting")}),
          experiment(StopTime = 36000, __Dymola_Algorithm = "Dassl"));
      end LoadRescuedByBattery;
    end UsingBatteries;

    package UsingDiesel "Examples using a Diesel"
      extends Icons.myExamplesPackage;

      model OneDieselRescuingOneLoad
        extends Icons.myExample;
        PowerSysPro.Components.myEmergencySource diesel(Smax = 6000, UNom = 400, rampTime = 30) annotation (
          Placement(visible = true, transformation(origin = {-90, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        PowerSysPro.Components.myLoad load(UNom = 400, P = 5000, Q = 100) annotation (
          Placement(visible = true, transformation(origin = {18, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        PowerSysPro.Components.myLine line(UNom = 400, Imax = 50, R = 0.5) annotation (
          Placement(visible = true, transformation(origin = {-4, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        PowerSysPro.Components.mySource src(UNom = 400) annotation (
          Placement(visible = true, transformation(origin = {-90, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        PowerSysPro.Components.myBreaker brk1(BrkOpen(start = true), UNom = 400) annotation (
          Placement(visible = true, transformation(extent = {{-66, 52}, {-46, 72}}, rotation = 0)));
        Modelica.Blocks.Logical.Not n annotation (
          Placement(visible = true, transformation(origin = {-114, 46}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        PowerSysPro.Components.myBreaker brk2(BrkOpen(start = true), UNom = 400) annotation (
          Placement(visible = true, transformation(extent = {{-54, 18}, {-34, 38}}, rotation = 0)));
        Modelica.Blocks.Sources.BooleanStep cmd(startTime = 30, startValue = true) annotation (
          Placement(visible = true, transformation(origin = {-142, 12}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      equation
        connect(line.terminalB, load.terminal) annotation (
          Line(points = {{6, 28}, {18, 28}, {18, 27.97}, {17.99, 27.97}}));
        connect(src.terminal, brk1.terminalA) annotation (
          Line(points = {{-90, 62}, {-66, 62}}));
        connect(brk1.terminalB, line.terminalA) annotation (
          Line(points = {{-46, 62}, {-14, 62}, {-14, 28}}));
        connect(diesel.terminal, brk2.terminalA) annotation (
          Line(points = {{-90, 28}, {-54, 28}}));
        connect(line.terminalA, brk2.terminalB) annotation (
          Line(points = {{-14, 28}, {-34, 28}}));
        connect(n.y, brk1.BrkOpen) annotation (
          Line(points = {{-107.4, 46}, {-56, 46}, {-56, 59}}, color = {255, 0, 255}));
        connect(cmd.y, brk2.BrkOpen) annotation (
          Line(points = {{-135.4, 12}, {-44, 12}, {-44, 25}}, color = {255, 0, 255}));
        connect(cmd.y, n.u) annotation (
          Line(points = {{-135.4, 12}, {-130, 12}, {-130, 46}, {-121.2, 46}}, color = {255, 0, 255}));
        annotation (
          Diagram(graphics={  Text(origin = {-44, 5}, lineColor = {28, 108, 200}, extent = {{-20, 69}, {232, 41}}, textString = "P=5 kW and Q=0.1 kvar for the load
      Pmax=6 kVA for the Diesel", fontSize = 12), Text(origin = {-17, -15}, lineColor = {28, 108, 200}, extent = {{-109, -17}, {128, -47}}, textString = "the breaker switches at 30 s
      when the Diesel is able to deliver its maximum power
      the load is supplied without any disturbance")}, coordinateSystem(initialScale = 0.1)),
          experiment(StopTime = 60, __Dymola_Algorithm = "Dassl"));
      end OneDieselRescuingOneLoad;

      model OneDieselRescuingOneLoadWithFailure_notFinalized
        extends Icons.myExample;
        PowerSysPro.Components.myEmergencySource diesel(Smax = 6000, UNom = 400, rampTime = 30) annotation (
          Placement(visible = true, transformation(origin = {-90, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        PowerSysPro.Components.myLoad load(UNom = 400, P = 5000, Q = 100) annotation (
          Placement(visible = true, transformation(origin = {18, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        PowerSysPro.Components.myLine line(UNom = 400, Imax = 50, R = 0.5) annotation (
          Placement(visible = true, transformation(origin = {-4, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        PowerSysPro.Components.mySource src(UNom = 400) annotation (
          Placement(visible = true, transformation(origin = {-90, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        PowerSysPro.Components.myBreaker brk1(BrkOpen(start = true), UNom = 400) annotation (
          Placement(visible = true, transformation(extent = {{-66, 52}, {-46, 72}}, rotation = 0)));
        Modelica.Blocks.Logical.Not n annotation (
          Placement(visible = true, transformation(origin = {-114, 46}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        PowerSysPro.Components.myBreaker brk2(BrkOpen(start = true), UNom = 400) annotation (
          Placement(visible = true, transformation(extent = {{-54, 18}, {-34, 38}}, rotation = 0)));
        Modelica.Blocks.Sources.BooleanStep cmd(startTime = 30, startValue = true) annotation (
          Placement(visible = true, transformation(origin = {-142, 12}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        Components.myFailure myFailure annotation (
          Placement(visible = true, transformation(origin = {-4, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(line.terminalB, load.terminal) annotation (
          Line(points = {{6, 28}, {18, 28}, {18, 27.97}, {17.99, 27.97}}));
        connect(src.terminal, brk1.terminalA) annotation (
          Line(points = {{-90, 62}, {-66, 62}}));
        connect(brk1.terminalB, line.terminalA) annotation (
          Line(points = {{-46, 62}, {-14, 62}, {-14, 28}}));
        connect(diesel.terminal, brk2.terminalA) annotation (
          Line(points = {{-90, 28}, {-54, 28}}));
        connect(line.terminalA, brk2.terminalB) annotation (
          Line(points = {{-14, 28}, {-34, 28}}));
        connect(n.y, brk1.BrkOpen) annotation (
          Line(points = {{-107.4, 46}, {-56, 46}, {-56, 59}}, color = {255, 0, 255}));
        connect(cmd.y, brk2.BrkOpen) annotation (
          Line(points = {{-135.4, 12}, {-44, 12}, {-44, 25}}, color = {255, 0, 255}));
        connect(cmd.y, n.u) annotation (
          Line(points = {{-135.4, 12}, {-130, 12}, {-130, 46}, {-121.2, 46}}, color = {255, 0, 255}));
        connect(cmd.y, myFailure.u) annotation (
          Line(points={{-135.4,12},{-66,12},{-66,-6},{-16,-6}},        color = {255, 0, 255}));
        annotation (
          Diagram(graphics={  Text(origin = {-44, 5}, lineColor = {28, 108, 200}, extent = {{-20, 69}, {232, 41}}, textString = "P=5 kW and Q=0.1 kvar for the load
            Pmax=6 kVA for the Diesel", fontSize = 12), Text(origin = {-17, -15}, lineColor = {28, 108, 200}, extent = {{-109, -17}, {128, -47}}, textString = "the breaker switches at 30 s
            when the Diesel is able to deliver its maximum power
            the load is supplied without any disturbance")}, coordinateSystem(initialScale = 0.1)),
          experiment(StopTime = 60, __Dymola_Algorithm = "Dassl"));
      end OneDieselRescuingOneLoadWithFailure_notFinalized;
    end UsingDiesel;
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
    version = "2.3.0",
    versionDate = "2021-11-26",
    Documentation(info = "<html><head></head><body>
    <p>Copyright © 2020-2021, EDF.</p>
    <p>The use of the PowerSysPro library is granted by EDF under the provisions of the Modelica License 2. A copy of this license can be obtained&nbsp;<a href=\"http://www.modelica.org/licenses/ModelicaLicense2\">here</a>.</p>
    <p></p><p>-------------------------------------------------------------------------------------------------------------</p>
<p>For further technical information, see the <a href=\"modelica://PowerSysPro.Information\">Information</a>.</p>
</body></html>"),
    Diagram(graphics={  Text(lineColor = {28, 108, 200}, extent = {{-174, 28}, {180, -28}}, fontSize = 14, textStyle = {TextStyle.Bold}, textString = "Open electrical library
developed at EDF Lab. Paris-Saclay")}),
    Icon(graphics={  Text(extent = {{-208, 70}, {214, -60}}, lineColor = {28, 108, 200}, fontName = "Segoe Print", textString = "PSP")}),
    uses(Modelica(version = "4.0.0")));
end PowerSysPro;

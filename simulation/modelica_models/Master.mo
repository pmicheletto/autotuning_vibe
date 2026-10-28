package Master
  extends Modelica;

  model SimplesPID
    Modelica.Blocks.Continuous.LimPID PI(Ni = 0.1, Td = 0.1, Ti = 0.1, controllerType = Modelica.Blocks.Types.SimpleController.PI, initType = Modelica.Blocks.Types.Init.SteadyState, k = 10, limiter(u(start = 0)), yMax = 12) annotation(
      Placement(transformation(origin = {0, 22}, extent = {{-56, -20}, {-36, 0}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia1(J = 1, a(fixed = true, start = 0), phi(final start = 0, fixed = true), w(start = 0, fixed = true)) annotation(
      Placement(transformation(origin = {0, 22}, extent = {{2, -20}, {22, 0}})));
    Modelica.Mechanics.Rotational.Sources.Torque torque(useSupport = false) annotation(
      Placement(transformation(origin = {0, 22}, extent = {{-25, -20}, {-5, 0}})));
    Modelica.Mechanics.Rotational.Components.SpringDamper spring(c = 1e4, d = 100, stateSelect = StateSelect.prefer, w_rel(fixed = true)) annotation(
      Placement(transformation(origin = {0, 22}, extent = {{32, -20}, {52, 0}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia2(J = 2, phi(displayUnit = "rad")) annotation(
      Placement(transformation(origin = {0, 22}, extent = {{60, -20}, {80, 0}})));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation(
      Placement(transformation(origin = {0, 22}, extent = {{22, -50}, {2, -30}})));
    Modelica.Mechanics.Rotational.Sources.ConstantTorque loadTorque(tau_constant = 10, useSupport = false) annotation(
      Placement(transformation(origin = {0, 22}, extent = {{98, -15}, {88, -5}})));
    Modelica.Blocks.Sources.Step step(startTime = 1, height = 50) annotation(
      Placement(transformation(origin = {-84, 12}, extent = {{-10, -10}, {10, 10}})));
  equation
    connect(spring.flange_b, inertia2.flange_a) annotation(
      Line(points = {{52, 12}, {60, 12}}));
    connect(inertia1.flange_b, spring.flange_a) annotation(
      Line(points = {{22, 12}, {32, 12}}));
    connect(torque.flange, inertia1.flange_a) annotation(
      Line(points = {{-5, 12}, {2, 12}}));
    connect(speedSensor.flange, inertia1.flange_b) annotation(
      Line(points = {{22, -18}, {22, 12}}));
    connect(loadTorque.flange, inertia2.flange_b) annotation(
      Line(points = {{88, 12}, {80, 12}}));
    connect(PI.y, torque.tau) annotation(
      Line(points = {{-35, 12}, {-27, 12}}, color = {0, 0, 127}));
    connect(speedSensor.w, PI.u_m) annotation(
      Line(points = {{1, -18}, {-46, -18}, {-46, 0}}, color = {0, 0, 127}));
    connect(step.y, PI.u_s) annotation(
      Line(points = {{-72, 12}, {-58, 12}}, color = {0, 0, 127}));
    annotation(
      Diagram(graphics = {Rectangle(origin = {0, 22}, lineColor = {255, 0, 0}, extent = {{-25, 6}, {99, -50}}), Text(origin = {0, 22}, textColor = {255, 0, 0}, extent = {{4, 14}, {71, 7}}, textString = "plant")}));
  end SimplesPID;

  model TransferFunction
  Modelica.Blocks.Sources.Step step(height = 50, startTime = 1) annotation(
      Placement(transformation(origin = {-46, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.TransferFunction transferFunction(b = {1}, a = {0.05, 1})  annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}})));
  equation
  connect(step.y, transferFunction.u) annotation(
      Line(points = {{-35, 0}, {-13, 0}}, color = {0, 0, 127}));
  end TransferFunction;
  annotation(
    uses(Modelica(version = "4.1.0")));
end Master;
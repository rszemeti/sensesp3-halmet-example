#ifndef ANALOG_RPM_INPUT_H
#define ANALOG_RPM_INTPUT_H

template<int D, const char* INPUT_NAME, const char* ENGINE_NAME, int INITIAL_SORT_ORDER>
void setupAnalogRPMOutput() {
  auto* enable_rpm_output =
      new CheckboxConfig(true, "Enable RPM Output", "/Tacho " INPUT_NAME "/Enabled");
  enable_rpm_output->set_description(
      "Enable RPM input " INPUT_NAME ". Requires a reboot to take effect.");
  enable_rpm_output->set_sort_order(INITIAL_SORT_ORDER);

  if (enable_rpm_output->get_value()) {
    // Connect the tacho senders. Engine name is "main".
    auto tacho_frequency =
        ConnectTachoSender(D, "Tacho " INPUT_NAME, ENGINE_NAME, INITIAL_SORT_ORDER + 100);

    tacho_frequency->connect_to(
        new SKOutput<float>("propulsion." ENGINE_NAME ".revolutions", "",
                            new SKMetadata("Hz", "Main Engine Revolutions")));

    auto* engine_hours = new TimeCounter<float>("/Tacho " INPUT_NAME "/Engine Hours");
    engine_hours->set_description(
        "Engine hours based on the " INPUT_NAME " tacho input, in seconds.");
    engine_hours->set_sort_order(INITIAL_SORT_ORDER + 400);
    tacho_frequency->connect_to(engine_hours);

    // Create and connect the engine hours output object
    engine_hours->connect_to(
        new SKOutput<float>("propulsion." ENGINE_NAME ".runTime", "",
                            new SKMetadata("s", "Main Engine running time")));

    // Create a propulsion state lambda transform
    auto* propulsion_state = new LambdaTransform<float, String>([](bool freq) {
      if (freq > 0) {
        return "started";
      } else {
        return "stopped";
      }
    });

    // Connect the tacho frequency to the propulsion state lambda transform
    tacho_frequency->connect_to(propulsion_state);
    // Create and connect the propulsion state output object
    propulsion_state->connect_to(new SKOutput<String>(
        "propulsion." ENGINE_NAME ".state", "", new SKMetadata("", "Main Engine State")));

    if (enable_n2k_output->get_value()) {
      // Connect outputs to the N2k senders.
      tacho_frequency->connect_to(
          &(n2k_engine_rapid_sender->engine_speed_consumer_));
    }

    if (display_present) {
      tacho_frequency->connect_to(new LambdaConsumer<float>(
          [](float value) { PrintValue(display, 3, "RPM " INPUT_NAME, 60 * value); }));
    }
  }
}

#endif // ANALOG_RPM_OUTPUT_H

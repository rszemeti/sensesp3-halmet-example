#ifndef ANALOG_TEMPERATURE_INPUT_H
#define ANALOG_TEMPERATURE_INPUT_H



template<int A, const char* INPUT_NAME, const char* TEMPERATURE_PATH, const char* RESISTANCE_PATH, const char* ALARM_PATH, int INITIAL_SORT_ORDER>
void setupAnalogTemperatureInput() {
  auto* enable_input =
      new CheckboxConfig(false, INPUT_NAME, TEMPERATURE_PATH "/Enabled");
  enable_input->set_description(
      "Enable analog temperature input " INPUT_NAME ". Requires a reboot to take effect.");
  enable_input->set_sort_order(INITIAL_SORT_ORDER);

  if (enable_input->get_value()) {
    // Connect the temperature sender.
    auto analog_resistance = ConnectAnalogSender(ads1115, A, INPUT_NAME);
    // Resistance converted to temperature in Celsius
    auto sender_temperature =
        (new CurveInterpolator(nullptr, TEMPERATURE_PATH "/Temperature"))
            ->set_input_title("Sender Resistance (ohms)")
            ->set_output_title("Temperature (C)");
    sender_temperature->set_description(
        "Piecewise linear conversion of the resistance of the " INPUT_NAME " sender to a "
        "temperature in Celsius. Input is resistance, output is temperature in "
        "Celsius.");
    sender_temperature->set_sort_order(INITIAL_SORT_ORDER + 100);
    if (sender_temperature->get_samples().empty()) {
      // If there's no prior configuration, provide a default curve
      sender_temperature->clear_samples();
      sender_temperature->add_sample(CurveInterpolator::Sample(0, 0));
      sender_temperature->add_sample(CurveInterpolator::Sample(180., 300));
      sender_temperature->add_sample(CurveInterpolator::Sample(300., 300));
    }
    analog_resistance->connect_to(sender_temperature);

    const ParamInfo* high_temperature_limit =
        new ParamInfo[1]{{"high_temperature_limit", "High Temperature Limit"}};

    auto alarm_function = [](float temperature, float limit) -> bool {
      return temperature > limit;
    };

    auto sender_high_temperature_alarm = new LambdaTransform<float, bool, float>(
        alarm_function,
        100.0f,              // Default value for parameter
        high_temperature_limit,  // Parameter UI description
        ALARM_PATH "/High Temperature Alarm");
    sender_high_temperature_alarm->set_description(
        "Alarm if the temperature exceeds the set limit. Value in Celsius.");
    sender_high_temperature_alarm->set_sort_order(INITIAL_SORT_ORDER + 150);
    sender_temperature->connect_to(sender_high_temperature_alarm);

#ifdef ENABLE_SIGNALK
    auto analog_resistance_sk_output =
        new SKOutputFloat(RESISTANCE_PATH,
                          TEMPERATURE_PATH "/Sender Resistance",
                          new SKMetadata("ohm", "Input " INPUT_NAME " sender resistance",
                                         "Input " INPUT_NAME " sender resistance"));
    analog_resistance_sk_output->set_sort_order(INITIAL_SORT_ORDER + 200);
    analog_resistance->connect_to(analog_resistance_sk_output);
    auto sender_temperature_sk_output = new SKOutputFloat(
        TEMPERATURE_PATH "/Current Temperature", TEMPERATURE_PATH "/Current Temperature",
        new SKMetadata("C", "Oil Temperature", "Main Engine Oil Temperature"));
    sender_temperature_sk_output->set_sort_order(INITIAL_SORT_ORDER + 250);
    sender_temperature->connect_to(sender_temperature_sk_output);
#endif
    if (enable_n2k_output->get_value()) {
      // Connect the temperature output to N2k dynamic sender
      sender_temperature->connect_to(
          &(n2k_engine_dynamic_sender->oil_temperature_consumer_));

      // Connect the high temperature alarm to N2k dynamic sender
      sender_high_temperature_alarm->connect_to(
          &(n2k_engine_dynamic_sender->high_oil_temperature_consumer_));
    }
  }  // if (enable_input->get_value())
}

#endif // ANALOG_TEMPERATURE_INPUT_H

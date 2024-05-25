#ifndef ANALOG_PRESSURE_INPUT_H
#define ANALOG_PRESSURE_INPUT_H

template<int A, const char* INPUT_NAME, const char* PRESSURE_PATH, const char* RESISTANCE_PATH, const char* ALARM_PATH, int INITIAL_SORT_ORDER>
void setupAnalogPressureInput() {
  auto* enable_input =
      new CheckboxConfig(false, INPUT_NAME, PRESSURE_PATH "/Enabled");
  enable_input->set_description(
      "Enable analog pressure input " INPUT_NAME ". Requires a reboot to take effect.");
  enable_input->set_sort_order(INITIAL_SORT_ORDER);

  if (enable_input->get_value()) {
    // Connect the pressure sender.
    auto analog_resistance = ConnectAnalogSender(ads1115, A, INPUT_NAME);
    // Resistance converted to pressure in Pascal
    auto sender_pressure =
        (new CurveInterpolator(nullptr, PRESSURE_PATH "/Pressure"))
            ->set_input_title("Sender Resistance (ohms)")
            ->set_output_title("Pressure (Pa)");
    sender_pressure->set_description(
        "Piecewise linear conversion of the resistance of the " INPUT_NAME " sender to a "
        "pressure in Pascal. Input is resistance, output is pressure in "
        "Pascal.");
    sender_pressure->set_sort_order(INITIAL_SORT_ORDER + 100);
    if (sender_pressure->get_samples().empty()) {
      // If there's no prior configuration, provide a default curve
      sender_pressure->clear_samples();
      sender_pressure->add_sample(CurveInterpolator::Sample(0, 0));
      sender_pressure->add_sample(CurveInterpolator::Sample(180., 300000));
      sender_pressure->add_sample(CurveInterpolator::Sample(300., 300000));
    }
    analog_resistance->connect_to(sender_pressure);

    const ParamInfo* low_pressure_limit =
        new ParamInfo[1]{{"low_pressure_limit", "Low Pressure Limit"}};

    auto alarm_function = [](float pressure, float limit) -> bool {
      return pressure < limit;
    };

    auto sender_low_pressure_alarm = new LambdaTransform<float, bool, float>(
        alarm_function,
        100000,              // Default value for parameter
        low_pressure_limit,  // Parameter UI description
        ALARM_PATH "/Low Pressure Alarm");
    sender_low_pressure_alarm->set_description(
        "Alarm if the pressure falls below the set limit. Value in Pascal.");
    sender_low_pressure_alarm->set_sort_order(INITIAL_SORT_ORDER + 150);
    sender_pressure->connect_to(sender_low_pressure_alarm);

#ifdef ENABLE_SIGNALK
    auto analog_resistance_sk_output =
        new SKOutputFloat(RESISTANCE_PATH,
                          PRESSURE_PATH "/Sender Resistance",
                          new SKMetadata("ohm", "Input " INPUT_NAME " sender resistance",
                                         "Input " INPUT_NAME " sender resistance"));
    analog_resistance_sk_output->set_sort_order(INITIAL_SORT_ORDER + 200);
    analog_resistance->connect_to(analog_resistance_sk_output);
    auto sender_pressure_sk_output = new SKOutputFloat(
        PRESSURE_PATH "/Current Pressure", PRESSURE_PATH "/Current Pressure",
        new SKMetadata("Pa", "Oil Pressure", "Main Engine Oil Pressure"));
    sender_pressure_sk_output->set_sort_order(INITIAL_SORT_ORDER + 250);
    sender_pressure->connect_to(sender_pressure_sk_output);
#endif
    if (enable_n2k_output->get_value()) {
      // Connect the pressure output to N2k dynamic sender
      sender_pressure->connect_to(
          &(n2k_engine_dynamic_sender->oil_pressure_consumer_));

      // Connect the low pressure alarm to N2k dynamic sender
      sender_low_pressure_alarm->connect_to(
          &(n2k_engine_dynamic_sender->low_oil_pressure_consumer_));
    }
  }  // if (enable_input->get_value())
}

#endif // ANALOG_PRESSURE_INPUT_H

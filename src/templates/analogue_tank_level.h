#ifndef ANALOG_TANK_LEVEL_INPUT_H
#define ANALOG_TANK_LEVEL_INPUT_H

template<int A, const char* INPUT_NAME, const char* TANK_PATH, const char* RESISTANCE_PATH, int INITIAL_SORT_ORDER>
void setupAnalogTankLevelInput() {
  auto* enable_input =
      new CheckboxConfig(false, INPUT_NAME, TANK_PATH "/Enabled");
  enable_input->set_description(
      "Enable analog tank level input " INPUT_NAME ". Requires a reboot to take effect.");
  enable_input->set_sort_order(INITIAL_SORT_ORDER);

  if (enable_input->get_value()) {
    // Connect the tank sender.
    auto analog_resistance = ConnectAnalogSender(ads1115, A, INPUT_NAME);
    // Resistance converted to relative value 0..1
    auto tank_level =
        (new CurveInterpolator(nullptr, TANK_PATH "/Level Curve"))
            ->set_input_title("Sender Resistance (ohms)")
            ->set_output_title("Fill Level (ratio)");
    tank_level->set_description(kFillLevelCurveDescription);
    tank_level->set_sort_order(INITIAL_SORT_ORDER + 100);
    if (tank_level->get_samples().empty()) {
      // If there's no prior configuration, provide a default curve
      tank_level->clear_samples();
      tank_level->add_sample(CurveInterpolator::Sample(0, 0));
      tank_level->add_sample(CurveInterpolator::Sample(180., 1));
      tank_level->add_sample(CurveInterpolator::Sample(300., 1));
    }
    analog_resistance->connect_to(tank_level);
    // Level converted to remaining volume in m3
    auto tank_volume =
        new Linear(kTankDefaultSize, 0, TANK_PATH "/Total Volume");
    tank_volume->set_description("Total volume of tank " INPUT_NAME " in m3");
    tank_volume->set_sort_order(INITIAL_SORT_ORDER + 200);
    tank_level->connect_to(tank_volume);

#ifdef ENABLE_SIGNALK
    auto analog_resistance_sk_output = new SKOutputFloat(
        RESISTANCE_PATH, TANK_PATH "/Sender Resistance",
        new SKMetadata("ohm", "Input " INPUT_NAME " sender resistance",
                       "Input " INPUT_NAME " sender resistance"));
    analog_resistance_sk_output->set_sort_order(INITIAL_SORT_ORDER + 300);
    analog_resistance->connect_to(analog_resistance_sk_output);
    auto tank_level_sk_output = new SKOutputFloat(
        TANK_PATH "/Current Level", TANK_PATH "/Current Level",
        new SKMetadata("ratio", "Tank " INPUT_NAME " level", "Tank " INPUT_NAME " level"));
    tank_level_sk_output->set_sort_order(INITIAL_SORT_ORDER + 400);
    tank_level->connect_to(tank_level_sk_output);
    auto tank_volume_sk_output = new SKOutputFloat(
        TANK_PATH "/Current Volume", TANK_PATH "/Current Volume",
        new SKMetadata("m3", "Tank " INPUT_NAME " volume",
                       "Calculated tank " INPUT_NAME " remaining volume"));
    tank_volume_sk_output->set_sort_order(INITIAL_SORT_ORDER + 500);
    tank_volume->connect_to(tank_volume_sk_output);
#endif

    if (enable_n2k_output->get_value()) {
      auto tank_n2k_level_output =
          new N2kFluidLevelSender(TANK_PATH "/NMEA 2000", A, N2kft_Water, 200,
                                  nmea2000, enable_n2k_output->get_value());
      tank_n2k_level_output->set_sort_order(INITIAL_SORT_ORDER + 600);
      tank_volume->connect_to(
          &(tank_n2k_level_output->tank_level_consumer_));
    }
  }  // if (enable_input->get_value())
}

#endif // ANALOG_TANK_LEVEL_INPUT_H

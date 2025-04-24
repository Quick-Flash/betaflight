# Provizora
The word Provizora comes from the Esperanto word for temporary/provisionary.
The name was picked because this project is meant to be provisional as we trasition to a fully Rust project.
This will lay the groundwork for the Rust project and will help launch its success.
But why an Esperanto word? 
Esperanto was created to be easy to use, easy to learn language made for the purpose of bringing others together through easy to understand communication.
I like that message and feel like this project, in the long term, can play the role of an easy to use firmware that can bring the drone world together.

# Helpful Starting Video
https://youtu.be/8qr64LfN-po

# Functional Differences From BF
- Anti-gravity only effects the pterm instead of the pterm and iterm.
- Improved sensor fusion method for dual gyro FC.
- AHRS (attitude estimation) uses raw instead of filtered gyro.
- Gyro lowpass filters replaced with predictive lowpass filters.
- Two tap arming by default.
- Soft arming by default, with a lower motor idle while in soft arming.
- Mixer learns the CG location and adds a direct compensation for it.
- Detection of collisions reducing the "freakout" the happens. Requires ACC to be enabled to work.
- If a full collision is detected at near 0 throttle, re-enable soft arming, makes landing simple.
- Custom mixer setups are now done differently (more information to come)
- Voltage compensation is always applied to the mixer (may make that optional later)
- Voltage compensation uses the `vbat_full_cell_voltage` and `vbat_min_cell_voltage` as the range it compensates for.
- Voltage compensation in the mixer can be configured to make throttle feel consistent over the flight `voltage_throttle_comp` (defaults to not have this behavior)
- Voltage compensation should make the drone respond more consistently over the whole battery voltage range.

## Setting Changes
| Setting Name           | BF Value | New Value | Description                                                                           |
|------------------------|----------|-----------|---------------------------------------------------------------------------------------|
| min_check              | 1050     | 1000      | Min rc range value                                                                    |
| max_check              | 1900     | 2000      | Max rc range value                                                                    |
| anti_gravity_gain      | 80       | 20        | Anti-gravity gain                                                                     |
| vbat_full_cell_voltage | 410      | 420       | Max voltage used for voltage mixer compensation, and for OSD warning of over voltage. |
| yaw_lowpass_hz         | 100      | 0         | Lowpass filter on yaw pterm (not generally needed)                                    |
| iterm_rotation         | Off      | On        | Better handling of iterm during rotations                                             |

## New Settings
| Setting Name                | Default Value | Description                                                                                                                                                |
|-----------------------------|---------------|------------------------------------------------------------------------------------------------------------------------------------------------------------|
| gyro_lpf1_cutoff            | 125           | First gyro lowpass filter cutoff                                                                                                                           |
| gyro_lpf1_variant           | PT1           | Filter type for the first gyro lowpass filter, more info below, search gyro_lpf1_variant                                                                   |
| gyro_lpf1_q                 | 707           | Q value for the gyro lowpass filter, more info below, search gyro_lpf1_q                                                                                   |
| gyro_lpf1_pred_cutoff       | 15            | First gyro lowpass filter cutoff for the predictive portion of the filter, more info below, search gyro_lpf1_pred_cutoff                                   |
| gyro_lpf1_pred_variant      | OFF           | Filter type for the first gyro lowpass filter, more info below, search gyro_lpf1_pred_variant                                                              |
| gyro_lpf1_pred_weight       | 100           | A weight from 0 to 100 for how much of the predictive portion of the filter is used                                                                        |
| gyro_lpf1_pred_q            | 707           | Q value for the predictive portion of the gyro lowpass filter, more info below, search gyro_lpf1_pred_q                                                    |
| gyro_lpf2_cutoff            | 160           | Second gyro lowpass filter cutoff                                                                                                                          |
| gyro_lpf2_variant           | PT1           | Filter type for the second gyro lowpass filter, more info below, search gyro_lpf1_variant                                                                  |
| gyro_lpf2_q                 | 707           | Q value for the gyro lowpass filter, more info below, search gyro_lpf1_q                                                                                   |
| gyro_lpf2_pred_cutoff       | 15            | First gyro lowpass filter cutoff for the predictive portion of the filter, more info below, search gyro_lpf1_pred_cutoff                                   |
| gyro_lpf2_pred_variant      | OFF           | Filter type for the first gyro lowpass filter, more info below, search gyro_lpf1_pred_variant                                                              |
| gyro_lpf2_pred_weight       | 100           | A weight from 0 to 100 for how much of the predictive portion of the filter is used                                                                        |
| gyro_lpf2_pred_q            | 707           | Q value for the predictive portion of the gyro lowpass filter, more info below, search gyro_lpf1_pred_q                                                    |
| gyro_noise_est_cut          | 15            | How quickly the gyro fusion will shift its prediction from one gyro to another                                                                             |
| cg_learning_time            | 20            | Time in tenths of a second that the CG will be learned from the iterm, lower values is faster learning                                                     |
| thrust_linear_low           | 60            | Thrust linear low like EmuFlight                                                                                                                           |
| thrust_linear_high          | 30            | Thrust linear high like EmuFlight                                                                                                                          |
| thrust_linear_cut           | 75            | Pt1 filter cutoff on the change that thrust linear makes, should help reduce low throttle thrust linear noise                                              |
| motor_cut_low               | 350           | Pt1 filter cutoff on the motor values at low throttle, think of this as a smarter throttle moving filter                                                   |
| motor_cut_high              | 750           | Pt1 filter cutoff on the motor values at high throttle, think of this as a smarter throttle moving filter                                                  |
| collision_jerk_start        | 550           | Amount of jerk (change in acceleration read from accelerometer) is needed to start detecting collisions                                                    |
| collision_jerk_end          | 750           | Amount of jerk (change in acceleration read from accelerometer) is needed to fully detect collisions                                                       |
| two_tap_arming              | 1             | If set to 1 flipping the arm switch twice in one second is required to arm, 0 disables this                                                                |
| soft_arm_throttle_threshold | 25            | Soft arming is enabled until above this throttle. As you raise throttle to this point you gain more authority. After this throttle soft arming is disabled |
| motor_soft_idle             | 300           | This is the motor idle value while in soft arming. Typically set to as low as the motor will reliable start to spin                                        |
| voltage_throttle_comp       | 0             | When set to 1 or on this setting will try to keep the throttle feeling the same as the battery voltage is drained.                                         |
| rpm_filter_pred_weight      | 100           | A weight from 0 to 100 for how much of the predictive portion of the RPM notch filter is used                                                              |

## Blackbox Changes
### `DEBUG_DUAL_GYRO` Gyro fusion debug
- Debug 0: first gyro roll  
- Debug 1: first gyro roll noise  
- Debug 2: second gyro roll  
- Debug 3: second gyro roll noise  
- Debug 4: first gyro pitch  
- Debug 5: first gyro pitch noise  
- Debug 6: second gyro pitch  
- Debug 7: second gyro pitch noise
### `CG_COMPENSATION` CG compensation debug
- Debug 0: estimated relative x CG * 1000
- Debug 1: estimated relative y CG * 1000
- Debug 2: motor 0 thrust gain * 1000 (CG Compensation changes this)
- Debug 3: motor 1 thrust gain * 1000 (CG Compensation changes this)
- Debug 4: motor 2 thrust gain * 1000 (CG Compensation changes this)
- Debug 5: motor 3 thrust gain * 1000 (CG Compensation changes this)
### `COLLISION_DETECTION` Collision detection debug
- Debug 0: measured jerk, or derivative/change of the accelerometer
- Debug 1: allowed mixer range for roll, pitch and yaw, multiplied by 1000.
  - A value of 1000 is full roll, pitch and yaw control.
  - A value of 0 is no roll, pitch and yaw control, only throttle control.
### `SOFT_ARM` Soft arming debug
- Debug 0: percentage of soft arming you are in.
- Debug 1: allowed mixer range for roll, pitch and yaw, multiplied by 1000 due to soft arming.
  - A value of 1000 is full roll, pitch and yaw control.
  - A value of 0 is no roll, pitch and yaw control, only throttle control.
- Debug 2: motor range min (motor output low after dynamic idle).
- Debug 3: throttle * 1000 (0 is 0 throttle and 100 is full throttle).
- Debug 4: motor 0 * 1000
- Debug 5: motor 1 * 1000
- Debug 6: motor 2 * 1000
- Debug 7: motor 3 * 1000

# Changelog

### 4/22/2025
- Use predictive notch filtering on the RPM filter.
- Should reduce latency, perhaps look at using a lower Q on the RPM filter.
- Uses the rpm_filter_pred_weight for how strong the predictive part is.

### 2/15/2025
- Update CG Compensation to use rotation rate as well.
- High rotation rates will lead to no learning occurring.

### 2/7/2025
- Add voltage compensation to the mixer.
- Voltage compensation is always applied to the mixer (may make that optional later)
- Voltage compensation uses the `vbat_full_cell_voltage` and `vbat_min_cell_voltage` as the range it compensates for.
- Voltage compensation in the mixer can be configured to make throttle feel consistent over the flight `voltage_throttle_comp` (defaults to not have this behavior)
- Voltage compensation should make the drone respond more consistently over the whole battery voltage range.
- CG compensation should be fixed!
- New `SOFT_ARM` debug mode.

### 1/26/2025
- Refactor entire mixer and replace with my own mixer.
- Mixer currently only supports 4 motor outputs.
- Mixer refactoring better support soft arming.
- Custom motor mixes are now handled slightly differently.
- New mixer settings:
  - `thrust_linear_low` with a default of 60
  - `thrust_linear_high` with a default of 30
    - thrust linear now works like EmuFlight https://github.com/emuflight/EmuFlight/wiki/Motor-Mixers-and-Thrust-Linearization
    - set both to 0 to disable.
  - `thrust_linear_cut` with a default of 75
    - This is the cutoff to a pt1 filter on the amount that thrust linear changes the throttle. This should help with low throttle noise that thrust linear amplifies on motors.
    - Set to 0 to disable.
  - `motor_cut_low` with a default of 350
  - `motor_cut_high` with a default of 750
    - Motor cut is a cutoff to a pt1 filter on the motor output themselves.
    - The cutoff shifts as the demanded motor output changes, with low occurring at 0 throttle and high at full throttle.
- RPM filter accuracy is increased.
- CG compensation is added, it uses Iterm and thrust to estimate CG.
- The CG is fed into the mixer and should better compensate for thrust imbalance.
- `anti_gravity_gain` is reduced from 80 to 20 by default (CG compensation handles this now).
- New CG Compensation settings:
  - `cg_learning_time` with a default of 20
    - Learning time is the time in tenths of a second that it takes the mixer to learn 70% of the CG.
    - As CG is learning Iterm is attenuated as CG Compensation largely replaces its role.
    - If your drone is tuned well without any Iterm related issues, lower values may make it respond better to outside stimulus.
- New CG Compensation debug:
  - `CG_COMPENSATION` is the new debug mode
  - Element 0 is the estimated relative x CG
  - Element 1 is the estimated relative y CG
  - Element 2 is motor 0 thrust gain (CG Compensation changes this)
  - Element 3 is motor 1 thrust gain (CG Compensation changes this)
  - Element 4 is motor 2 thrust gain (CG Compensation changes this)
  - Element 5 is motor 3 thrust gain (CG Compensation changes this)
- Collision Detection is added and uses the accelerometer to determine if you have collided with anything.
- When a collision is detected the mixer will begin to only allow for thrust, and no pitch, roll or yaw.
- New Collision Detection settings:
  - `collision_jerk_start` with a default of 350
  - `collision_jerk_end` with a default of 550
    - The derivative of the accelerometer (jerk) is used to determine when we have collided.
    - The amount that the mixer allows for pitch, roll, and yaw is determined by how much jerk is measured.
    - When the jerk becomes higher than `collision_jerk_start` it will start to limit them, and at `collision_jerk_end` it will completely limit them.
    - If the jerk is greater than the `collision_jerk_end` and throttle is low (less than 1%) soft arming is re-enabled.
- New Collision Detection debug:
  - `COLLISION_DETECTION` is the new debug mode
  - Element 0 is the measured jerk, or derivative of the accelerometer
  - Element 1 is the allowed mixer range for roll, pitch and yaw, multiplied by 1000.
    - A value of 1000 is full roll, pitch and yaw control.
    - A value of 0 is no roll, pitch and yaw control, only throttle control.
- All the new settings are available in the CLI or OSD under the new mixer tab of pid tuning.
- BF version bump to 4.6.4

### 10/24/2024
- Major lowpas filtering overhaul!
- Removal of Predictive filter variants, now all lowpass filters can be optionally made predictive.
- Lowpass filter variants are now `"OFF", "PT1", "PT2", "PT3", "FIRST_ORDER", "SECOND_ORDER", "SLIDING_ORDER", "PT_SECOND_ORDER"`
- `gyro_lpf1_variant` is for the main filter
- `gyro_lpf1_pred_variant` is for the predictive portion of the filter. If you set this to `OFF` then there will be no predictive part to the filter and it will act like normal.
- `gyro_lpf1_pred_weight` is how much of the prediction you want to use. It is set as a percent, with 0 disabling the prediction and 100 being full predictive strength.
- `SLIDING_ORDER` is a lowpass filter that can be configured to be between a `FIRST_ORDER` and `SECOND_ORDER` filter. It does make use of `Q`. The setting `gyro_lpf1_cutoff_shift` determine how much like a `FIRST_ORDER` or `SECOND_ORDER` filter it is. A value of 0 is like a `FIRST_ORDER`, and a value of 100 is like a `SECOND_ORDER`. This applies for the predictive part as well.
- `PT_SECOND_ORDER` is a lowpass filter that can be configured to be between a `PT1` and `SECOND_ORDER` filter. It does make use of `Q`. The setting `gyro_lpf1_cutoff_shift` determine how much like a `FIRST_ORDER` or `SECOND_ORDER` filter it is. A value of 0 is like a `PT1`, and a value of 100 is like a `SECOND_ORDER`. This applies for the predictive part as well.
- BF version bump to 4.6.1

### 10/22/2024
- Prevent lowpass filters from being set above nyquist and causing issues.
- Fix blackbox logging (probably) and add the new lowpass settings to blackbox.
- make the AHRS use raw gyro input instead of filtered gyro, this should make the attitude estimation more accurate.

### 10/20/2024
- Remove the BF gyro lowpass filters and replace with my own.
- Added a median filter as a downsampling filter to remove BF's lpf2 downsampling
- Add lowpass filter options of off, pt1, pt2, pt3, first order, second order, predictive pt1, predictive first order, predictive second order.
- Setting cutoff to 0 will not set the filter to off, you instead need to set the filter variant to off
- All new settings are found in the OSD under the gyro filters tab
- All predictive filters are a lowpass filter with another lowpass filter on the error between the filter and the raw input. Example, if your raw input is 50.1, but the filter output is 25.1, then the error would be 25.0 and there would be an additional filter being fed that value. It then adds its output together with the main lowpass filter. This "predictive" part is able to remove latency at the cost of a little additional noise at lower frequencies. The higher the predictive cutoff the less latency, but more noise you will have.
- Below are the new settings:
  - `gyro_lpf1_cutoff` -> Filter cutoff for the first lowpass filter
  - `gyro_lpf1_pred_cutoff` -> The cutoff for the predictive part of the filter. Higher values will remove latency faster, but may also lead to more noise.
  - `gyro_lpf1_q` -> Q value for the lowpass filter. Only applies to second order filters (both predictive and non predictive). A value of 707 will mimic a BF "biquad" filter. Higher values is less latent with a bit less filtering around cutoff, and lower values are more latent with more filtering around cutoff. Values greater than 707 will start to increase noise just below the cutoff.
  - `gyro_lpf1_pred_q` -> Q value for the predictive part of the lowpass filter. Only applies to the predictive second order filter
  - `gyro_lpf1_variant` -> Sets which filter variant you will be using. pt1, pt2, pt3 are what you expect them to be. first order is a lowpass filter that filters slightly more than a pt1, mainly at very high frequencies. second order is the BF biquad filter.
- Options are the same for the `gyro_lpf2`

### 10/15/2024
- Remove iterm from anti-gravity, rejoice!
- Modify `DEBUG_DUAL_GYRO` to display the sensor fusion. 

Debug 0: first gyro roll  
Debug 1: first gyro roll noise  
Debug 2: second gyro roll  
Debug 3: second gyro roll noise  
Debug 4: first gyro pitch  
Debug 5: first gyro pitch noise  
Debug 6: second gyro pitch  
Debug 7: second gyro pitch noise

### 10/13/2024
- Remove gyro overflow protection (sounds scary but few gyros implement this). Removed as I don't plan on implementing this logic into the Rust PID loop logic.
- For FC with dual gyros of the same type added smarter sensor fusion. Adding one new parameter `gyro_noise_est_cut`. 
It can be found in the OSD labeled `GYRO NOISE CUT` in the `FILTER GLB`.
This parameter is a filter on the "noise" of each gyro. The more "noisy" gyro will be weighted less when fusing the gyros together. 
A lower cutoff will mean that the weighting will switch between gyros slower.
- TODO modify the existing `DEBUG_DUAL_GYRO_DIFF` to make it useful for viewing the noise of each sensor.

![Betaflight](images/bf_logo.png)

[![Latest version](https://img.shields.io/github/v/release/betaflight/betaflight)](https://github.com/betaflight/betaflight/releases) [![Build](https://img.shields.io/github/actions/workflow/status/betaflight/betaflight/nightly.yml?branch=master)](https://github.com/betaflight/betaflight/actions/workflows/nightly.yml) [![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0) [![Join us on Discord!](https://img.shields.io/discord/868013470023548938)](https://discord.gg/n4E6ak4u3c)

Betaflight is flight controller software (firmware) used to fly multi-rotor craft and fixed wing craft.

This fork differs from Baseflight and Cleanflight in that it focuses on flight performance, leading-edge feature additions, and wide target support.

## Events

| Date  | Event |
| - | - |
| 28-04-2024 | Firmware 4.5 Release |


## News

### Requirements for the submission of new and updated targets

The following new requirements for pull requests adding new targets or modifying existing targets are put in place from now on:

1. Read the [hardware specification](https://betaflight.com/docs/development/manufacturer/manufacturer-design-guidelines)

2. No new F3 based targets will be accepted;

3. For any new target that is to be added, only a Unified Target config into https://github.com/betaflight/unified-targets/tree/master/configs/default needs to be submitted. See the [instructions](https://betaflight.com/docs/manufacturer/creating-an-unified-target) for how to create a Unified Target configuration. If there is no Unified Target for the MCU type of the new target (see instructions above), then a 'legacy' format target definition into `src/main/target/` has to be submitted as well;

4. For changes to existing targets, the change needs to be applied to the Unified Target config in https://github.com/betaflight/unified-targets/tree/master/configs/default. If no Unified Target configuration for the target exists, a new Unified Target configuration will have to be created and submitted. If there is no Unified Target for the MCU type of the new target (see instructions above), then an update to the 'legacy' format target definition in `src/main/target/` has to be submitted alongside the update to the Unified Target configuration.


## Features

Betaflight has the following features:

* Multi-color RGB LED strip support (each LED can be a different color using variable length WS2811 Addressable RGB strips - use for Orientation Indicators, Low Battery Warning, Flight Mode Status, Initialization Troubleshooting, etc)
* DShot (150, 300 and 600), Multishot, Oneshot (125 and 42) and Proshot1000 motor protocol support
* Blackbox flight recorder logging (to onboard flash or external microSD card where equipped)
* Support for targets that use the STM32 F4, G4, F7 and H7 processors
* PWM, PPM, SPI, and Serial (SBus, SumH, SumD, Spektrum 1024/2048, XBus, etc) RX connection with failsafe detection
* Multiple telemetry protocols (CRSF, FrSky, HoTT smart-port, MSP, etc)
* RSSI via ADC - Uses ADC to read PWM RSSI signals, tested with FrSky D4R-II, X8R, X4R-SB, & XSR
* OSD support & configuration without needing third-party OSD software/firmware/comm devices
* OLED Displays - Display information on: Battery voltage/current/mAh, profile, rate profile, mode, version, sensors, etc
* In-flight manual PID tuning and rate adjustment
* PID and filter tuning using sliders
* Rate profiles and in-flight selection of them
* Configurable serial ports for Serial RX, Telemetry, ESC telemetry, MSP, GPS, OSD, Sonar, etc - Use most devices on any port, softserial included
* VTX support for Unify Pro and IRC Tramp
* and MUCH, MUCH more.

## Installation & Documentation

See: https://betaflight.com/docs/wiki

## Support and Developers Channel

There's a dedicated Discord server here:

https://discord.gg/n4E6ak4u3c

We also have a Facebook Group. Join us to get a place to talk about Betaflight, ask configuration questions, or just hang out with fellow pilots.

https://www.facebook.com/groups/betaflightgroup/

Etiquette: Don't ask to ask and please wait around long enough for a reply - sometimes people are out flying, asleep or at work and can't answer immediately.

## Configuration Tool

To configure Betaflight you should use the Betaflight-configurator GUI tool (Windows/OSX/Linux) which can be found here:

https://github.com/betaflight/betaflight-configurator/releases/latest

## Contributing

Contributions are welcome and encouraged. You can contribute in many ways:

* implement a new feature in the firmware or in configurator (see [below](#Developers));
* documentation updates and corrections;
* How-To guides - received help? Help others!
* bug reporting & fixes;
* new feature ideas & suggestions;
* provide a new translation for configurator, or help us maintain the existing ones (see [below](#Translators)).

The best place to start is the Betaflight Discord (registration [here](https://discord.gg/n4E6ak4u3c)). Next place is the github issue tracker:

https://github.com/betaflight/betaflight/issues
https://github.com/betaflight/betaflight-configurator/issues

Before creating new issues please check to see if there is an existing one, search first otherwise you waste people's time when they could be coding instead!

If you want to contribute to our efforts financially, please consider making a donation to us through [PayPal](https://paypal.me/betaflight).

If you want to contribute financially on an ongoing basis, you should consider becoming a patron for us on [Patreon](https://www.patreon.com/betaflight).

## Developers

Contribution of bugfixes and new features is encouraged. Please be aware that we have a thorough review process for pull requests, and be prepared to explain what you want to achieve with your pull request.
Before starting to write code, please read our [development guidelines](https://betaflight.com/docs/development) and [coding style definition](https://betaflight.com/docs/development/CodingStyle).

GitHub actions are used to run automatic builds

## Translators

We want to make Betaflight accessible for pilots who are not fluent in English, and for this reason we are currently maintaining translations into 21 languages for Betaflight Configurator: Català, Dansk, Deutsch, Español, Euskera, Français, Galego, Hrvatski, Bahasa Indonesia, Italiano, 日本語, 한국어, Latviešu, Português, Português Brasileiro, polski, Русский язык, Svenska, 简体中文, 繁體中文.
We have got a team of volunteer translators who do this work, but additional translators are always welcome to share the workload, and we are keen to add additional languages. If you would like to help us with translations, you have got the following options:
- if you help by suggesting some updates or improvements to translations in a language you are familiar with, head to [crowdin](https://crowdin.com/project/betaflight-configurator) and add your suggested translations there;
- if you would like to start working on the translation for a new language, or take on responsibility for proof-reading the translation for a language you are very familiar with, please head to the Betaflight Discord chat (registration [here](https://discord.gg/n4E6ak4u3c)), and join the ['translation'](https://discord.com/channels/868013470023548938/1057773726915100702) channel - the people in there can help you to get a new language added, or set you up as a proof reader.

## Hardware Issues

Betaflight does not manufacture or distribute their own hardware. While we are collaborating with and supported by a number of manufacturers, we do not do any kind of hardware support.
If you encounter any hardware issues with your flight controller or another component, please contact the manufacturer or supplier of your hardware, or check RCGroups https://rcgroups.com/forums/showthread.php?t=2464844 to see if others with the same problem have found a solution.

## Betaflight Releases

https://github.com/betaflight/betaflight/releases

## Open Source / Contributors

Betaflight is software that is **open source** and is available free of charge without warranty to all users.

Betaflight is forked from Cleanflight, so thanks goes to all those who have contributed to Cleanflight and its origins.

Origins for this fork (Thanks!):
* **Alexinparis** (for MultiWii),
* **timecop** (for Baseflight),
* **Dominic Clifton** (for Cleanflight),
* **borisbstyle** (for Betaflight), and
* **Sambas** (for the original STM32F4 port).

The Betaflight Configurator is forked from Cleanflight Configurator and its origins.

Origins for Betaflight Configurator:
* **Dominic Clifton** (for Cleanflight configurator), and
* **ctn** (for the original Configurator).

Big thanks to current and past contributors:
* Budden, Martin (martinbudden)
* Bardwell, Joshua (joshuabardwell)
* Blackman, Jason (blckmn)
* ctzsnooze
* Höglund, Anders (andershoglund)
* Ledvina, Petr (ledvinap) - **IO code awesomeness!**
* kc10kevin
* Keeble, Gary (MadmanK)
* Keller, Michael (mikeller) - **Configurator brilliance**
* Kravcov, Albert (skaman82) - **Configurator brilliance**
* MJ666
* Nathan (nathantsoi)
* ravnav
* sambas - **bringing us the F4**
* savaga
* Stålheim, Anton (KiteAnton)

And many many others who haven't been mentioned....

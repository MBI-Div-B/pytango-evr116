# pytango-EVR116

## Description
With this TangoDS you can control the [Pfeiffer EVR116 motorized gas valve](https://www.pfeiffer-vacuum.com/de/produkte/ventile-kammern-komponenten/vakuumventile/druckregelventile/gasregelventile/1796/evr-116-gasregelventil-motorisch) via VacuPI.

## DAC calibration 
Please note that each hardware instance need to be calibrated with `dac_calibration` command in `EXPERT` mode. Please read the README attribute and its doc properly before running the calibartion. Ask [@lrlunin](https://github.com/lrlunin) if something is unclear.

## Usage
1. Assign an either target voltage or gas flow to one of tango attributes.
2. Execute `apply()` command to apply the changes to the valve.

## Authors

Contributors names and contact info

[@lrlunin](https://github.com/lrlunin)

## License

This project is licensed under the MIT License - see the LICENSE.md file for details
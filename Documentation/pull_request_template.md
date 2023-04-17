# Pull Request Template

## General Guidelines for Titling Pull Requests

Pull request (PR) titles are important when generating release notes for the MSDK. In general, if a label exists relating to the PR's changes, the template should go like:

`[LABEL]: [Description]`

### Description

Please use a concise description in the titles that are appropriate for release notes. This includes proper capitalization and no run-on sentences. PR titles that are all lower case are not acceptable.

### Examples

##### Microcontroller-specific Titles

Use the target label.

`MAX32XXX: [Description]` or `MAX78XXX:  [Description]`

`MAX32690: Added new example.`

`MAX32655, MAX78000: Updated ADC Registers.`

##### Peripheral-specific Titles

Use the "API Change" label if necessary. There are no labels pertaining to each peripheral. You can specify the peripheral library's revision.

`[PERIPH]: [Description]`

`TMR: Changed clock source enums.`

`I2C: Check for both interrupt flag registers in handler.`

`ADC_RevA: Added assertion to all Monitor Functions.`

`TMR, HTMR, WUT: Added external clock support.`

##### Bug Fixes

Use the "bug" label.

`[Ticket #]: [Description]`

`[Ticket #123]: Fixed build system.`

##### BLE-specific Titles

Use the "BLE" label.

`BLE: [Description]`

`BLE: Switched PAL Timers to use WUT for MAX32665.`

##### Tool-specific Titles

Use the "Tools" label.

Template:
`Tools: [Description]`

`Tools: Updated max32690.cfg.`

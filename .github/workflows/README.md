# Workflows

## Generate_Register_Files

### How to use the Workflow on GitHub

This workflow will allow users to generate the register files for all supported parts in the msdk repo. This is a PR comment-triggered action. As the user, you just need to update and push in your peripheral SVD file changes, then comment with the command below in the PR to get a runner to push in your updated register files AND the svd schema (max32xxx.svd or max78xxx.svd). The workflow will find parts linked to your pushed peripheral SVD file changes accordingly.

`/generate-register-files`

Note: This command will only update the parts that are directly impacted by any peripheral SVD file changes in your PR.

If needed, you can run the command below to generate register files or the svd schema for a single part:

`/generate-register-files (PART_DESIGNATION)`

The `(PART_DESIGNATION)` parameter refers to the part number: max32655, ME10, MAX78002, me18. Capitalization doesn't matter. For example:

`/generate-register-files me15` or `/generate-register-files MAX32690`


You can also update the register files and svd schema for all parts using the command below. This command will explicitly update every parts' register files and svd schema even if there were no changes to a parts' peripheral SVD file within the PR. This may be useful when the SVD scripts are updated and you want to implement a new formatting change into all the register files. 

`/generate-register-files all`

#### Reading the Status of Generate_Register_Files Action

The runner will react with a 'rocket' emoji once it starts the job, with a 'hooray' emoji when the job is successful, and with a 'confused' emoji if the job fails. This is just a quality-of-life feature, so you don't have to go into the Actions tab to see the status of this job.

### Updating the Peripheral SVD Files.

The register files and SVD Schema for each part are located in **msdk/Libraries/CMSIS/Device/Maxim/{PART}/Include/**, and the peripheral SVD files are usually located in **msdk/Libraries/PeriphDrivers/Source/{PERIPHERAL}**.

It's recommended to use an XML Editor to easily view and make changes in the SVD files. [XML Notepad](https://microsoft.github.io/XmlNotepad/#) by Microsoft is a good one.

To update a certain register file, you will have to find the [SVD files in the msdk-internal repo](https://github.com/Analog-Devices-MSDK/msdk-internal/tree/main/SVD/Devices) and go into the parts' **chip_periph.txt** to find the register files' corresponding peripheral SVD file. Follow the naming convention key below to help you find the correct SVD file.

Key:

    {periph}_regs.h   <-> {periph}_revX.svd or {periph}_revX_{part}.svd
    adc_regs.h        <-> adc_reva.svd or adc_reva_{part}.svd
    gpio_regs.h       <-> gpio_reva.svd or gpio_reva_{part}.svd
    uart_regs.h       <-> uart_revb.svd or uart_revb_{part}.svd
    gcr_regs.h        <-> gcr_{part}.svd
    fcr_regs.h        <-> fcr_{part}.svd

Note: The SYS Register Files are chip-specific, so each part will have their own SYS SVD file: `fcr_{part}.svd`, `gcr_{part}.svd`, `mcr_{part}.svd`, `sir_{part}.svd`, and `trimsir_{part}.svd` to name a few.

Once you found the correct peripheral SVD file(s), push in your changes and run the workflow. You should be good to go.

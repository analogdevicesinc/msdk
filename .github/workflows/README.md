# Workflows

## Generate_Register_Files

### How to use the Workflow on GitHub

This workflow will allow users to generate the register files for all supported parts in the msdk repo. This is a PR comment-triggered action. As the user, you just need to update and push in your peripheral SVD file changes, then comment with the command below in the PR to get a runner to push in your updated register files AND the svd schema (max32xxx.svd or max78xxx.svd). The workflow will find parts linked to your pushed peripheral SVD file changes accordingly.

`/generate-register-files`

The runner will react with a 'rocket' emoji once it starts the job, with a 'hooray' emoji when the job is succeeds, and with a 'confused' emoji if the job failes


To generate register files for a single part, then you can run the command below:

`/generate-register-files (PART_DESIGNATION)`

Where (PART_DESIGNATION) refers to the part number: max32655, ME10, MAX78002, me18. Capitalization doesn't matter. For example:

`/generate-register-files me15` or `/generate-register-files MAX32690`


To generate register files for all parts, then you can run the command below:

`/generate-register-files all`

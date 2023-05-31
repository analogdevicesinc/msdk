## Pull Request Template

### Overview

The MSDK repo follows a custom rule-set based on the [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) standard. The pull request titles will follow this format:

`type(scope): Subject`

#### Types

The list of possible types are:

1.  **fix** – a bug fix has occurred
2.  **feat** – a new feature was introduced
3.  **chore** – changes that do not relate to a fix or feature, or a catch-all type for changes that don’t really fit under any of the other types
4.  **refactor** – refactored code that neither fixes a bug nor adds a new feature
5.  **docs** – updates to documentation
6.  **style** – code styling changes (linter or clang-format)
7.  **test** – changes involving test code (not changes like deleting a line of code to test if your application works or not)
8.  **perf** – performance improvements
9.  **ci** – changes related to continuous integration
10. **build** – changes that affect build system, tool chain, or any other external dependencies
11. **revert** – reverts a commit

Appending an `!` at the end of the type signifies the PR is a breaking or major change.

NOTE: The type is case sensitive and must match one of the listed types.

#### Scopes

The scopes are dependent on the changes based on their location in the MSDK. This list of possible scopes are:

1.  **BLE** – Any bluetooth related changes (e.g. Cordio)
2.  **Documentation** – Any changes in msdk/Documentation folder
3.  **Examples** – Any changes in msdk/Examples
4.  **Tools** – Any changes in msdk/Tools
5.  **Boards** – Any board file changes
6.  **CMSIS** – Any CMSIS/Register file changes
7.  **MiscDrivers** – Any MiscDrivers changes (e.g. TFT Display, PB, LED, External Flash Drivers)
8.  **PeriphDrivers** – Any changes in msdk/Libraries/PeriphDrivers
9.  **Third-Party** – All third-party libraries (e.g. FreeRTOS, LVGL, lwIP)
10. **ignore** – Small and quick miscellaneous fixes (should not be used often)
11. **workflow** – Any GitHub workflow related changes

NOTE: The scope is case sensitive and must match one of the listed scopes.

### Responsibilites for Reviewers/Mergers/PR Owners

These are the rules for the `Subject`.

1.  Use imperative mood in the Subjects - meaning the tone of the Subject should be like giving an order or request.
    * “Add” instead of “Added” or “Adding”.
    * “Fix” instead of “Fixed” or “Fixing”.
    * “Update” instead of “Updated” or “Updating”.
    * “Delete” instead of “Deleted” or “Deleting”.
2.  The Subject contents should have useful and concise wording.
    * No long descriptive subjects - not concise if it gets too descriptive. 
        * You can be more descriptive in the body or footer of the commit by running:
        `git commit -m “Title contents” -m “Body contents” -m “Footer contents”` 
    * No simple titles like “Add small SPI change”
3.  The Subject should include what parts are affected, if any. Only the parts’ CHIP NAME should be used in the Subject. No die names are allowed.
    * For the most readability, append the affected parts list at the end of the Subject, so the important, beginning part of the commit message is visible when traversing through the MSDK repo on GitHub.
    * The list of chip names should be in ascending numerical order in the Subject.
    * Examples:
        1. `fix(CMSIS): Rename TRIMSIR registers for MAX32670 and MAX32675`
        2. `fix(Third-Party): Remove USB DMA support for MAX32655, MAX32665, MAX32690, MAX78000, and MAX78002`
        3. `feat(Examples): Add console output in Hello_World READMEs for all parts`
        4. `fix(Examples,PeriphDrivers): Deprecate MXC\_RTC\_GetSecond and MXC\_RTC\_GetSubSecond for all parts except for MAX32520`
            * Use this type of wording if all but a few parts were affected to shorten the Subject.
    * **TIP**: The auto-labeler workflow adds part labels to a PR depending on what specific files were updated. Use the list of labels to figure out what parts were affected by the PR's changes.
4.  If the PR title is related to a Jira ticket or an issue, the Subject should begin with the ticket designation.
    * Examples:
        1. `fix(CMSIS): MSDK-123: Add missing I2C Target registers for MAX32670, MAX32672, and MAX32675` 
        2. `fix(PeriphDrivers): Ticket-321: Fix SPI hanging on 9-bit wide messages for all parts` 
5.  When a merger “Squash and Merges” an approved PR to main, do not delete the PR number that is automatically appended to the title. Ideally, the PR title should be following our rules before approval, so the merger just needs to press the “Squash and Merge” button.
    * Using previous examples from rule 4, this is what the commit should look like in the main branch:
        1. `fix(CMSIS): MSDK-123: Add missing I2C Target registers for all parts (#412)`
        2. `fix(PeriphDrivers): Ticket-321: Fix SPI hanging on 9-bit wide messages for all parts (#646)`
        3. `feat(Examples): Add console output in Hello_World READMEs for all parts (#342)`
        4. `fix(Examples,PeriphDrivers): Deprecate MXC\_RTC\_GetSecond and MXC\_RTC\_GetSubSecond for all parts except for MAX32520 (#248)`

### PR Format Rules (Workflow Enforced)
1.  The type is capital-sensitive and must be one of the listed types.
2.  The scope is capital-sensitive and must be one of the listed scopes.
3.  The first word of the Subject must be capitalized.
4.  The Subject must not end with any punctuation (periods, commas, exclamation marks).

### Linked Tickets

(Optional) Link any related Github issues [using a keyword](https://docs.github.com/en/issues/tracking-your-work-with-issues/linking-a-pull-request-to-an-issue#linking-a-pull-request-to-an-issue-using-a-keyword)

### Key Changes

(Optional) Provide a link to any specific commits that you want to highlight.  This is especially useful in larger PRs.

### Tests

(Optional) Provide info on any relevant functional testing/validation.  For API changes or significant features this is not optional.

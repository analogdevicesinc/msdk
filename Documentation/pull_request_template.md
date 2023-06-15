## Pull Request Template

### Description

Please include a summary of the changes and related issue. Please also include relevant motivation and context.

### Checklist Before Requesting Review

- [ ] PR Title follows correct guidelines.
- [ ] Description of changes and all other relevant information.
- [ ] (Optional) Link any related Github issues [using a keyword](https://docs.github.com/en/issues/tracking-your-work-with-issues/linking-a-pull-request-to-an-issue#linking-a-pull-request-to-an-issue-using-a-keyword)
- [ ] (Optional) Provide info on any relevant functional testing/validation.  For API changes or significant features, this is not optional.

### PR Title Guidelines

The MSDK repo follows a custom rule-set based on the [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) standard. The pull request titles will follow this format:

`type(scope)<!>: Subject`

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

Appending an `!` at the end of the title prefix signifies the PR is a breaking or major change. The <!> is optional in the PR format.

NOTE: The type is case sensitive and must match one of the listed types.

#### Scopes

The scopes are dependent on the changes based on their location in the MSDK. The list of possible scopes are:

1.  **BLE** – Any bluetooth related changes (i.e. Cordio)
2.  **Documentation** – Any changes in msdk/Documentation folder
3.  **Examples** – Any changes in msdk/Examples
4.  **Tools** – Any changes in msdk/Tools
5.  **Boards** – Any board file changes
6.  **CMSIS** – Any CMSIS/Register file changes
7.  **MiscDrivers** – Any MiscDrivers changes (e.g. TFT Display, PB, LED, External Flash Drivers)
8.  **PeriphDrivers** – Any changes in msdk/Libraries/PeriphDrivers
9.  **ThirdParty** – Any third-party library changes (e.g. FreeRTOS, LVGL, lwIP)
10. **MAXUSB** – Any MAXUSB changes
11. **SDHC** – Any SDHC changes
12. **ignore** – Small and quick miscellaneous fixes (should not be used often)
13. **workflow** – Any GitHub workflow related changes
14. **Other** – Any changes that may not fit in the other scopes

NOTE: The scope is case sensitive and must match one of the listed scopes.

#### Subject Rules

1.  Use imperative mood in the Subjects - meaning the tone of the Subject should be like giving an order or request.
2.  The Subject should have useful and concise wording.
3.  The Subject should include what parts are affected in numerical order, if any. Only the parts’ CHIP NAME should be used in the Subject. No die names are allowed.
4.  If the PR title is related to a Jira ticket or an issue, the Subject should begin with the ticket designation.
5.  The first letter must be capitalized and the subject must not end with any punctuation.

#### More Info

Please refer to the [CONTRIBUTING.md](https://github.com/Analog-Devices-MSDK/msdk/blob/main/Documentation/CONTRIBUTING.md) for more information regarding the PR Title.

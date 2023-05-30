module.exports = { 
    /**
     * Resolve and load @commitlint/config-conventional from node_modules.
     * Referenced packages must be installed
     */
    extends: [
        "@commitlint/config-conventional"
    ],

    /**
     * Any rules defined here will override rules from @commitlint/config-conventional
     */
    rules: {
        "scope-enum": [2, "always", ["BLE", "MAX32520", "MAX32570", "MAX32572", "MAX32650", "MAX32655", "MAX32660", "MAX32662", "MAX32665", "MAX32670", "MAX32672", "MAX32675", "MAX32680", "MAX32690", "MAX78000", "MAX78002", "Register", "Tools", "Workflow", "VS Code", "Installer", "IAR", "Keil", "ignore"]],
        "scope-empty": [2, "never"],        
        "type-enum": [2, "always", ["build", "chore", "ci", "docs", "feat", "fix", "perf", "refactor", "revert", "style", "test"]],
        "subject-case": [2, "always", ["sentence-case", "start-case", "pascal-case", "upper-case"]],
        "body-case": [2, "always", ["sentence-case", "start-case", "pascal-case", "upper-case"]],
    }

};
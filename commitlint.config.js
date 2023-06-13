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
        "scope-enum": [2, "always", ["Documentation", "Examples", "Tools", "BLE", "Boards", "CMSIS", "MiscDrivers", "PeriphDrivers", "ThirdParty", "SDHC", "MAXUSB", "ignore", "workflow", "Other"]],
        "scope-empty": [2, "never"],        
        "type-enum": [2, "always", ["build", "chore", "ci", "docs", "feat", "fix", "perf", "refactor", "revert", "style", "test"]],
        "subject-case": [2, "always", ["sentence-case", "start-case", "pascal-case", "upper-case"]],
        "body-case": [2, "always", ["sentence-case", "start-case", "pascal-case", "upper-case"]],
    }

};
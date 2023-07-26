# MISRA Compliance

The AWS SigV4 Library files conform to the [MISRA C:2012](https://www.misra.org.uk)
guidelines, with some noted exceptions. Compliance is checked with Coverity static analysis.
The specific deviations, suppressed inline, are listed below.

Additionally, [MISRA configuration file](https://github.com/aws/SigV4-for-AWS-IoT-embedded-sdk/blob/main/tools/coverity/misra.config) contains the project wide deviations.

### Suppressed with Coverity Comments
To find the deviation references in the source files run grep on the source code
with ( Assuming rule 5.4 violation; with justification in point 1 ):
```
grep 'MISRA Ref 5.4.1' . -rI
```

#### Rule 5.4
_Ref 5.4.1_

- MISRA Rule 5.4 flags the following macro's name as ambiguous from the
        one postfixed with _LENGTH. The macro highlighted by the deviation is already
        in use and changing the name will break existing user projects. Thus, for
        backwards compatibility, the macro is not modified and kept as is and the
        deviation is suppressed.

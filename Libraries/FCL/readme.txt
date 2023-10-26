This software package provides ECDSA signatures computations and verifications
for secp192r1, secp256r1, spec384r1, secp521r1, bp256r1, bp384r1 and bp512r1 curves, for SHA-256, SHA-384, SHA-512 and SIA-256 hash functions.
This software is only here for test and demo purposes.
This implementation is not secure and does not protect the ECC private key.
The random number generator is not robust and will generate the same numbers at each startup (PRNG).
**** For that reason, do not use production-grade private keys ****

API hypertext documentation is in the docs/ folder

On a x86-cygwin PC, just run build_fcl_x86_external

release notes
1.0.0: initial release
1.0.1: p192 initializers size corrected; hash functions number updated
1.1.0: sha-3 (sha-224,sha-256,sha-384, sha-512) has been added
1.2.0: secp384r1, secp521r1, bp256r1, bp384r1, bp512r1 curves added; sha384 and sha512 added
1.2.1: AES (128,192, 256) in ECB, CBC modes and AES-CBC-MAC added

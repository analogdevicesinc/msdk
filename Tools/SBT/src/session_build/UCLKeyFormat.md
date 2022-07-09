
# UCL Key Format

## RSA

The type is rsa-pubkey-file.
This text file contains two lines:
• 512-digit hexadecimal-coded modulus
• 8-digit hexadecimal-coded public exponent, corresponding to the value 65537 10 i.e.
4 th Fermat number F4,

### Private key File

```
ASCII Encoded Hexadecimal MODULUS
ASCII Encoded Hexadecimal PRIVATE EXPONANT
ASCII Encoded Hexadecimal PUBLIC EXPONANT
```


### Public key File

The type is `rsa-pubkey-file`.
This text file contains two lines:

* 512-digit hexadecimal-coded modulus
* 8-digit hexadecimal-coded public exponent, corresponding to the value 65537 10 i.e. 4 th Fermat number F4,


### Example

```
98b26e6ca070831df979d735233109589ff99837b8f8737ee7a1bf402d5fdc0b1034
18bf546cfa9f2817f56fec3936c45f289d233a4b9b034a005d96afb400ca4472555e
67a9e68ae61f293795aff2808f52f55d01edeb741647b2c7a78dc51e5c85f281f2e8
053cbcc4804ea6219b9e0e0526a52747e68b04919eae85a750cf499318c6bfeb2022
38ac2e7b16e0811c7b2379ebb4c5256c6768e64d4933c736cbf8d6f49079a5a65012
8798bb6ea42626e79c710b9eb569e3bdb56ad170128a585346ffab8e6de9528bcff6
4aaace3622c189d16d39efacd73f1e2aca5648b10f134b3ebb3210e411fc65efab32
099519ad8264382c6c8b4e852035ccef35ad
00010001
```
The corresponding decimal value for the modulus above is:
19276210130415710777890506019644417995338073773931171559427350531879
31602279882499976190364819598026893963782172439692466590746029095374
86669370854976167832664191752150072649124705828171781137474190698714
39108687881747331743941931394625684555928899022276055867203171963675
34812114552040759967587402745550506618065819174108178398699263303378
44312677078500264815587702836700017318581240614179257660159839666795
69368429308206494485277064023456592466534521655927791440548162382157
43581743784026811226164039197387568849524007336444385570383685415595
29280988948233272641814263279021743851157631870385424245930119632749
33677


## ECDSA

32-byte value for
the secret,32-byte for the x affine coordinate, 32-byte value for the y affine coordinate

The type is ecdsa-privkey-file.

### Private key File

The file use the `.key` extenstion. It is a TEXT file formated as below :

```
ASCII Encoded Hexadecimal Secret d
ASCII Encoded Hexadecimal X affine coordinate
ASCII Encoded Hexadecimal Y affine coordinate
```


### Public key File

The file use the `.key` extenstion. It is a TEXT file formated as below :

```
ASCII Encoded Hexadecimal X affine coordinate
ASCII Encoded Hexadecimal Y affine coordinate
```

### Example:

For ECDSA NIST P-256 Private key :

```
7ac88a77095ce13e593b83904064f98351df9ed430eb143c4abc55a984e57f39
a823c8857948dc688f3a3ef3f6f220a514f05c2c6c1cef8c9f2f8df11dcf0142
3be124619cbbeb51e985328e8e33d321cade19628cc0db43304a7b27f2db8efe
```

The corresponding decimal value for the secret above is:
```
55536492590329705918454650842314976437754898324927855150797273542048128270137
```

The corresponding public key:

```
a823c8857948dc688f3a3ef3f6f220a514f05c2c6c1cef8c9f2f8df11dcf0142
3be124619cbbeb51e985328e8e33d321cade19628cc0db43304a7b27f2db8efe
```

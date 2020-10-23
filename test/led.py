import sys

def main():
    _sof_length = 4  # SOF bytes
    _eof_length = 4  # EOF bytes
    buffer_length = 4
    _brightness = 1

    _buf = []

    for _ in range(_sof_length):
        _buf.append(0b00000000)

    _buf.append(0b11100000 | int(_brightness * 31))
    _buf.append(0b00000000)
    _buf.append(0b11111111)
    _buf.append(0b00000000)

    for _ in range(_eof_length):
        _buf.append(0b11111111)

    for byte in _buf:
        _write_byte(byte)

def _write_byte(byte):
    for _ in range(8):
        sys.stdout.write("%d" % ((byte & 0x80) > 0))
        byte <<= 1
    print ""

if __name__ == '__main__':
    main()
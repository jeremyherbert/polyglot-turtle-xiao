
def calculate_register_from_frequency_arith(refclk, oversampling, baud_freq):
    assert oversampling in [16, 8, 3]
    return 65536 * (1 - oversampling * (baud_freq/refclk))


def calculate_frequency_from_register_arith(refclk, oversampling, register):
    return (refclk/oversampling) * (1 - (register/65536))


def calculate_error_at_frequency_arith(refclk, oversampling, baud_freq):
    register = calculate_register_from_frequency_arith(refclk, oversampling, baud_freq)
    actual_frequency = calculate_frequency_from_register_arith(refclk, oversampling, int(register))

    return 1 - (baud_freq / actual_frequency)


def calculate_register_from_frequency_frac(refclk, oversampling, fp, baud_freq):
    assert oversampling in [16, 8, 3]
    assert 0 <= fp <= 7
    return (refclk / (oversampling * baud_freq)) - fp/8


def calculate_frequency_from_register_frac(refclk, oversampling, fp, register):
    assert oversampling in [16, 8, 3]
    assert 0 <= fp <= 7
    return refclk / (oversampling * register + (fp / 8))


def calculate_error_at_frequency_frac(refclk, oversampling, fp, baud_freq):
    register = calculate_register_from_frequency_frac(refclk, oversampling, fp, baud_freq)
    actual_frequency = calculate_frequency_from_register_frac(refclk, oversampling, fp, int(register))

    return 1 - (baud_freq / actual_frequency)


if __name__ == "__main__":
    for freq in [1200, 2400, 4800, 9600, 19200, 384000, 57600, 115200, 128000, 230400, 256000, 460800, 500000, 675000, 1000000]:

        refclk = 48e6
        fp = 0

        if 1200 <= freq <= 10000:
            oversampling = 16
        elif 10000 < freq <= 500000:
            oversampling = 8
        elif 500000 < freq:
            oversampling = 3
        else:
            raise ValueError

        print("baud:", freq)
        print("\toversampling:", oversampling)
        print("\tarith",
              "; register:", int(calculate_register_from_frequency_arith(refclk, oversampling, freq)),
              "; error:", calculate_error_at_frequency_arith(refclk, oversampling, freq) * 100, "%")

        if oversampling >= 8:
            for fp in range(8):
                print("\tfp:", fp,
                      "; register:", int(calculate_register_from_frequency_frac(refclk, oversampling, fp, freq)),
                      "; error:", calculate_error_at_frequency_frac(refclk, oversampling, fp, freq) * 100, "%")


        print()

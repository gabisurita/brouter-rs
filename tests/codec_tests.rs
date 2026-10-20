//! Unit tests ported directly from:
//! - Java test: `btools.codec.StatCoderContextTest`
//! - Java source: `https://github.com/abrensch/brouter/blob/master/brouter-codec/src/test/java/btools/codec/StatCoderContextTest.java`

use brouter_rs::rd5::codec::bit_coder::StatCoderContext;

#[test]
fn test_noisy_var_bits_encode_decode() {
    let mut ab = vec![0u8; 40000];
    {
        let mut ctx = StatCoderContext::new(&mut ab);
        for noisybits in 1..12 {
            for i in 0..1000 {
                ctx.encode_noisy_number(i, noisybits);
            }
        }
        ctx.bit_coder.close_and_get_encoded_length();
    }

    let mut ctx = StatCoderContext::new(&mut ab);
    for noisybits in 1..12 {
        for i in 0..1000 {
            let value = ctx.decode_noisy_number(noisybits);
            assert_eq!(
                value, i,
                "value mismatch: noisybits={} i={} value={}",
                noisybits, i, value
            );
        }
    }
}

#[test]
fn test_noisy_signed_var_bits_encode_decode() {
    let mut ab = vec![0u8; 80000];
    {
        let mut ctx = StatCoderContext::new(&mut ab);
        for noisybits in 0..12 {
            for i in -1000..1000 {
                ctx.encode_noisy_diff(i, noisybits);
            }
        }
        ctx.bit_coder.close_and_get_encoded_length();
    }

    let mut ctx = StatCoderContext::new(&mut ab);
    for noisybits in 0..12 {
        for i in -1000..1000 {
            let value = ctx.decode_noisy_diff(noisybits);
            assert_eq!(
                value, i,
                "value mismatch: noisybits={} i={} value={}",
                noisybits, i, value
            );
        }
    }
}

#[test]
fn test_predicted_value_encode_decode() {
    let mut ab = vec![0u8; 80000];
    {
        let mut ctx = StatCoderContext::new(&mut ab);
        let mut value = -100;
        while value < 100 {
            let mut predictor = -200;
            while predictor < 200 {
                ctx.encode_predicted_value(value, predictor);
                predictor += 7;
            }
            value += 5;
        }
        ctx.bit_coder.close_and_get_encoded_length();
    }

    let mut ctx = StatCoderContext::new(&mut ab);
    let mut value = -100;
    while value < 100 {
        let mut predictor = -200;
        while predictor < 200 {
            let decoded = ctx.decode_predicted_value(predictor);
            assert_eq!(
                decoded, value,
                "value mismatch: value={} predictor={} decoded={}",
                value, predictor, decoded
            );
            predictor += 7;
        }
        value += 5;
    }
}

#[test]
fn test_sorted_array_encode_decode() {
    let size = 100_000;
    let mut values = Vec::with_capacity(size);

    // Simple deterministic LCG pseudo-random generator
    let mut seed = 42u64;
    for _ in 0..size {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        values.push(((seed >> 33) & 0x0fff_ffff) as i32);
    }
    values[5] = 175384;
    values[8] = 175384;
    values[15] = 275384;
    values[18] = 275385;

    values.sort_unstable();

    let mut ab = vec![0u8; 3_000_000];
    {
        let mut ctx = StatCoderContext::new(&mut ab);
        ctx.encode_sorted_array(&values, 0, size, 0x0800_0000, 0);
        ctx.bit_coder.close_and_get_encoded_length();
    }

    let mut ctx = StatCoderContext::new(&mut ab);
    let mut decoded = vec![0i32; size];
    ctx.decode_sorted_array(&mut decoded, 0, size, 27, 0);

    assert_eq!(
        decoded, values,
        "Decoded sorted array did not match original"
    );
}

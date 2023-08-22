use crate::cptv_encoder::bit_cursor::BitCursor;

/// CPTV files are contained inside a gzip stream.  To create spec-compliant CPTV files for this
/// embedded use-case, we support a very limited sub-set of the DEFLATE algorithm, which it turns
/// out works well for our data streams.
/// We have a pre-computed 'dynamic' deflate block with a known huffman table at the start which
/// captures the frequency distribution of the symbols typically seen in our delta-frame-encoded
/// CPTV files. Because the data is quite noisy, it's not worth having an LZ entropy encoder that
/// supports (length, distance) matches on earlier bytes of the input, especially given the additional
/// memory usage and code complexity that this entails for an embedded implementation.
/// Instead, we only output literal bytes which are then encoded using a corresponding huffman code.
/// Having less symbols in the alphabet than if we supported (length, distance) matching turns out
/// to be a win in most cases, since the subsequent shorter huffman codes for literals (0..=255)
/// actually outweigh any gains from supporting a complex matching system.

fn write_to_flash(cursor: &mut BitCursor, storage: &mut [u8; 4096]) {
    let (to_flush, num_bytes) = cursor.flush();
    for byte in &to_flush[0..num_bytes] {
        storage.push(*byte);
    }
}

fn write_gzip_header(cursor: &mut BitCursor, storage: &mut [u8; 4096]) {
    let gzip_header = [0x1f, 0x8b, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff];
    for byte in &gzip_header {
        if cursor.write_byte(*byte) {
            write_to_flash(cursor, storage);
        }
    }
}
fn write_dynamic_final_block_header(cursor: &mut BitCursor, storage: &mut [u8; 4096]) {
    // Deflate header indicating that it's the final block, 2 bits indicating it's a dynamic block,
    // followed immediately by the huffman table description used for decoding the block.
    let block_header_and_huffman_description = [
        5, 224, 3, 152, 109, 105, 182, 6, 106, 190, 255, 24, 107, 70, 196, 222, 153, 89, 85, 231,
        182, 109, 219, 182, 109, 219, 182, 109, 219, 182, 109, 219, 182, 109, 119, 223, 83, 200,
        220, 59, 34, 230, 26, 227, 235, 7,
    ];
    for byte in
        &block_header_and_huffman_description[..block_header_and_huffman_description.len() - 1]
    {
        if cursor.write_byte(*byte) {
            write_to_flash(cursor, storage);
        }
    }
    if cursor.write_bits(
        block_header_and_huffman_description[block_header_and_huffman_description.len() - 1] as u16,
        5,
    ) {
        write_to_flash(cursor, storage);
    }
}

fn write_uncompressed_final_block_header(cursor: &mut BitCursor, storage: &mut [u8; 4096]) {
    // TODO: write final block, uncompressed, then a trailer of length/nlength
}

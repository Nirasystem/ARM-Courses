import sys
from pydub import AudioSegment
import struct
import textwrap

def resample(audio, target_sample_rate):
    return audio.set_frame_rate(target_sample_rate)

def change_bits_per_sample(audio, target_bits_per_sample):
    return audio.set_sample_width(target_bits_per_sample // 8)

def extract_raw_data(audio, channels):
    if channels == 1:
        return (audio.split_to_mono()[0].raw_data, None)
    elif channels == 2:
        # If stereo, concatenate the raw data of both channels
        return (audio.split_to_mono()[0].raw_data, audio.split_to_mono()[1].raw_data)
    else:
        raise ValueError("Unsupported number of channels. Only 1 (mono) or 2 (stereo) are supported.")

def get_c_type(bits_per_sample):
    if bits_per_sample == 8:
        return "uint8_t"
    elif bits_per_sample == 16:
        return "uint16_t"
    elif bits_per_sample == 24 or bits_per_sample == 32:
        return "uint32_t"
    else:
        raise ValueError("Unsupported bits per sample. Only 8, 16, 24, or 32 are supported.")
    
def bytes_to_uint16(byte_array):
    # Ensure that the length of the byte array is even
    if len(byte_array) % 2 != 0:
        raise ValueError("Input byte array length must be even for conversion to halfwords.")

    # Use the struct module to unpack the bytes into halfwords
    p = struct.unpack(f"{len(byte_array) // 2}H", byte_array)
    return p

def bytes_to_uint24(byte_array):
    # Ensure that the length of the byte array is even
    if len(byte_array) % 3 != 0:
        raise ValueError("Input byte array length must be even for conversion to halfwords.")

    # Use the struct module to unpack the bytes into halfwords
    p = struct.unpack(f"{len(byte_array) // 3}H", byte_array)
    return p

def bytes_to_uint32(byte_array):
    # Ensure that the length of the byte array is even
    if len(byte_array) % 4 != 0:
        raise ValueError("Input byte array length must be even for conversion to halfwords.")

    # Use the struct module to unpack the bytes into halfwords
    p = struct.unpack(f"{len(byte_array) // 4}H", byte_array)
    return p

def rescale_tuple(original_tuple, max_value):
    rescaled_tuple = tuple(int(value * max_value / max(original_tuple)) for value in original_tuple)
    return rescaled_tuple

def save_as_c_file(raw_data, output_file, array_name, bits_per_sample):
    (left , right) =  raw_data
    c_type = get_c_type(bits_per_sample)

    # Combine bytes into the appropriate C data type
    if bits_per_sample == 8:
        left = rescale_tuple(left, 4095)
        hex_string_left = ', '.join([f'0x{byte:02X}' for byte in left])
        hex_string_left = textwrap.fill(hex_string_left, width=16*6, subsequent_indent=' ' * 2)
    elif bits_per_sample == 16:
        values = bytes_to_uint16(left)
        values = rescale_tuple(values, 4095)
        hex_string_left = ', '.join([f'0x{value:04X}' for value in values])
        hex_string_left = textwrap.fill(hex_string_left, width=16*8, subsequent_indent=' ' * 2)
    elif bits_per_sample == 24:
        values = bytes_to_uint24(left)
        values = rescale_tuple(values, 4095)
        hex_string_left = ', '.join([f'0x{value:06X}' for value in values])
        hex_string_left = textwrap.fill(hex_string_left, width=16*10, subsequent_indent=' ' * 2)
    elif bits_per_sample == 32:
        values = bytes_to_uint32(left)
        values = rescale_tuple(values, 4095)
        hex_string_left = ', '.join([f'0x{value:08X}' for value in values])
        hex_string_left = textwrap.fill(hex_string_left, width=16*12, subsequent_indent=' ' * 2)
    else:
        raise ValueError("Bits per sample rate not supported")
    
    if right != None:
        if bits_per_sample == 8:
            left = rescale_tuple(left, 4095)
            hex_string_right = ', '.join([f'0x{byte:02X}' for byte in right])
            hex_string_right = textwrap.fill(hex_string_right, width=16*6, subsequent_indent=' ' * 2)
        elif bits_per_sample == 16:
            values = bytes_to_uint16(right)
            values = rescale_tuple(values, 4095)
            hex_string_right = ', '.join([f'0x{value:04X}' for value in values])
            hex_string_right = textwrap.fill(hex_string_right, width=16*8, subsequent_indent=' ' * 2)
        elif bits_per_sample == 24:
            values = bytes_to_uint24(right)
            values = rescale_tuple(values, 4095)
            hex_string_right = ', '.join([f'0x{value:06X}' for value in values])
            hex_string_right = textwrap.fill(hex_string_right, width=16*10, subsequent_indent=' ' * 2)
        elif bits_per_sample == 32:
            values = bytes_to_uint32(right)
            values = rescale_tuple(values, 4095)
            hex_string_right = ', '.join([f'0x{value:08X}' for value in values])
            hex_string_right = textwrap.fill(hex_string_right, width=16*12, subsequent_indent=' ' * 2)
        else:
            raise ValueError("Bits per sample rate not supported")
    else:
        hex_string_right = None

    with open(output_file, 'w') as file:
        file.write(f"#include <stdint.h>\n\n")
        if hex_string_right != None:
            file.write(f"const {c_type} {array_name}_LEFT[] = {{\n  {hex_string_left} \n}};\n\n")
            file.write(f"const {c_type} {array_name}_RIGHT[] = {{\n  {hex_string_right} \n}};\n")
            file.write(f"const uint32_t {array_name}_LEN = sizeof({array_name}_LEFT) / sizeof({array_name}_LEFT[0]);\n")
        else:
            file.write(f"const {c_type} {array_name}[] = {{\n  {hex_string_left} \n}};\n\n")
            file.write(f"const uint32_t {array_name}_LEN = sizeof({array_name}) / sizeof({array_name}[0]);\n")
        

def main():
    if len(sys.argv) != 7:
        print("Usage: python audio_processing.py input_file output_file array_name sample_rate bits_per_sample channels")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    array_name = sys.argv[3]
    target_sample_rate = int(sys.argv[4])
    target_bits_per_sample = int(sys.argv[5])
    channels = int(sys.argv[6])

    # Load audio file
    audio = AudioSegment.from_file(input_file)

    # Resample audio
    audio = resample(audio, target_sample_rate)

    # Change bits per sample
    audio = change_bits_per_sample(audio, target_bits_per_sample)

    # Extract raw data
    raw_data = extract_raw_data(audio, channels)

    # Save as C file
    save_as_c_file(raw_data, output_file, array_name, target_bits_per_sample)

    print("Processing complete.")

if __name__ == "__main__":
    main()

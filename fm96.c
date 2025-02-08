#include <math.h>
#include <dsp.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <pulse/simple.h>
#include <pulse/error.h>

int sample_rate = 192000;
pa_simple *input_device;
pa_simple *output_device;
pa_simple *mpx_device;

#define buffer_maxlength 12288
#define buffer_tlength_fragsize 12288
#define buffer_prebuf 32
#define BUFFER_SIZE 768

volatile sig_atomic_t running = 1;

void stop(int signum) {
    (void)signum;
    printf("\nReceived stop signal.\n");
    running = 0;
}

void uninterleave(const float *input, float *left, float *right, size_t num_samples) {
    for (size_t i = 0; i < num_samples/2; i++) {
        left[i] = input[i * 2];
        right[i] = input[i * 2 + 1];
    }
}

void show_version() {
    printf("fm96 (an FM Processor by radio95) version 1.0\n");
}

int main(int argc, char **argv) {
    show_version();

    int stereo = 1;
    char *audio_input_device = "default";
    char *audio_output_device = "default";
    char *audio_mpx_device = "";

    // #region Parse Arguments
    const char *short_opt = "s:S:i:o:m:h";
    struct option long_opt[] = {
        {"stereo", required_argument, NULL, 's'},
        {"sample_rate", required_argument, NULL, 'S'},
        {"input", required_argument, NULL, 'i'},
        {"output", required_argument, NULL, 'o'},
        {"mpx", required_argument, NULL, 'm'},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0}
    };
    int opt;
    while ((opt = getopt_long(argc, argv, short_opt, long_opt, NULL)) != -1) {
        switch (opt) {
            case 's':
                stereo = atoi(optarg);
                break;
            case 'S':
                sample_rate = atoi(optarg);
                break;
            case 'i':
                audio_input_device = optarg;
                break;
            case 'o':
                audio_output_device = optarg;
                break;
            case 'm':
                audio_mpx_device = optarg;
                break;
            case 'h':
                printf("Usage: %s [OPTIONS]\n", argv[0]);
                printf("Options:\n");
                printf("  -s, --stereo=BOOL\tSet stereo mode (default: 1)\n");
                printf("  -S, --sample_rate=RATE\tSet the sample rate (default: 192000)\n");
                printf("  -i, --input=DEVICE\tSet the input device\n");
                printf("  -o, --output=DEVICE\tSet the output device\n");
                printf("  -m, --mpx=DEVICE\tSet the MPX input device\n");
                printf("  -h, --help\t\tShow this help message\n");
                printf("  -v, --version\t\tShow version information\n");
                return 0;
        }
    }
    // #endregion

    // #region Setup devices
    pa_sample_spec mono_format = {
        .format = PA_SAMPLE_FLOAT32NE,
        .channels = 1,
        .rate = sample_rate
    };
    pa_sample_spec stereo_format = {
        .format = PA_SAMPLE_FLOAT32NE,
        .channels = 2,
        .rate = sample_rate
    };
    pa_buffer_attr buffer_attr = {
        .maxlength = buffer_maxlength,
        .tlength = buffer_tlength_fragsize,
        .prebuf = buffer_prebuf
    };

    int error;
    output_device = pa_simple_new(NULL, "FM96", PA_STREAM_PLAYBACK, audio_output_device, "MPX Output", &mono_format, NULL, &buffer_attr, &error);
    if (!output_device) {
        fprintf(stderr, "pa_simple_new() failed: %s\n", pa_strerror(error));
        return 1;
    }

    input_device = pa_simple_new(NULL, "FM96", PA_STREAM_RECORD, audio_input_device, "Audio In", &stereo_format, NULL, &buffer_attr, &error);
    if (!input_device) {
        fprintf(stderr, "pa_simple_new() failed: %s\n", pa_strerror(error));
        pa_simple_free(output_device);
        return 1;
    }

    if(strlen(audio_mpx_device) != 0) {
        mpx_device = pa_simple_new(NULL, "FM96", PA_STREAM_RECORD, audio_mpx_device, "MPX In", &mono_format, NULL, &buffer_attr, &error);
        if (!mpx_device) {
            fprintf(stderr, "pa_simple_new() failed: %s\n", pa_strerror(error));
            pa_simple_free(input_device);
            pa_simple_free(output_device);
            return 1;
        }
    }

    // #endregion

    // #region Setup Filters/Modulaltors/Oscillators
    float pilot_sample;
    Oscillator pilot;
    init_sine_oscillator(&pilot, &pilot_sample, 19000, sample_rate);

    float lpf_preemp_l, lpf_preemp_r;
    float lpf_out_l, lpf_out_r;
    float preemp_input_l, preemp_input_r;

    BiquadFilter lpf_l, lpf_r;
    init_lpf(&lpf_l, &lpf_preemp_l, &lpf_out_l, 15000, 5.0f, sample_rate);
    init_lpf(&lpf_r, &lpf_preemp_r, &lpf_out_r, 15000, 5.0f, sample_rate);

    BiquadFilter preemp_l, preemp_r;
    init_preemphasis(&preemp_l, &preemp_input_l, &lpf_preemp_l, 50e-6, sample_rate);
    init_preemphasis(&preemp_r, &preemp_input_r, &lpf_preemp_r, 50e-6, sample_rate);
    // #endregion

    signal(SIGINT, stop);
    signal(SIGTERM, stop);

    float audio_stereo_input[BUFFER_SIZE*2];
    float left[BUFFER_SIZE+64], right[BUFFER_SIZE+64];
    float mpx_in[BUFFER_SIZE] = {0};
    float output[BUFFER_SIZE];
    while(running) {
        if (pa_simple_read(input_device, audio_stereo_input, sizeof(audio_stereo_input), &error) < 0) {
            fprintf(stderr, "Error reading from input device: %s\n", pa_strerror(error));
            running = 0;
            break;
        }
        if(strlen(audio_mpx_device) != 0) {
            if (pa_simple_read(mpx_device, mpx_in, sizeof(mpx_in), &error) < 0) {
                fprintf(stderr, "Error reading from MPX device: %s\n", pa_strerror(error));
                running = 0;
                break;
            }
        }
        uninterleave(audio_stereo_input, left, right, BUFFER_SIZE*2);

        for (int i = 0; i < BUFFER_SIZE; i++) {
            compute_sine_oscillator_sin(&pilot);
            float pilot_s = pilot_sample;
            compute_sine_oscillator_sin_multiplier(&pilot, 2);
            float stereo_carrier = pilot_sample;
            advance_sine_oscillator(&pilot);

            preemp_input_l = left[i];
            preemp_input_r = right[i];
            apply_biquad(&preemp_l);
            apply_biquad(&preemp_r);
            apply_biquad(&lpf_l);
            apply_biquad(&lpf_r);

            float mono = (lpf_out_l + lpf_out_r) / 2.0f;
            float stereo = (lpf_out_l - lpf_out_r) / 2.0f;

            if(stereo) {
                output[i] = mono*0.45f + pilot_s*0.09f + (stereo*stereo_carrier)*0.45f;
            } else {
                output[i] = mono;
            }
            output[i] += mpx_in[i];
        }

        if (pa_simple_write(output_device, output, sizeof(output), &error) < 0) {
            fprintf(stderr, "Error writing to output device: %s\n", pa_strerror(error));
            running = 0;
            break;
        }
    }

    exit:
    pa_simple_free(input_device);
    pa_simple_free(output_device);
    return 0;
}
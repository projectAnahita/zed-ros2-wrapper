#!/bin/bash

# Device names and profiles
MIC_SOURCE="alsa_input.usb-Audio_Technica_Crop_ATR2100x-USB_Microphone_12345678ABCD-00.analog-stereo"
SPEAKER_SINK="alsa_output.usb-EDIFIER_EDIFIER_G2000_EDI00000X07-01.analog-stereo"

# Function to setup echo cancellation
setup_audio() {
    echo "Setting up audio devices..."
    
    # Save initial state
    INITIAL_SOURCE=$(pactl get-default-source)
    INITIAL_SINK=$(pactl get-default-sink)
    
    # Load echo cancellation with known working settings
    echo "Setting up echo cancellation..."
    # pactl load-module module-echo-cancel source_name=$MIC_SOURCE sink_name=$SPEAKER_SINK channels=1 rate=16000 aec_method=webrtc aec_args="analog_gain_control=0,digital_gain_control=1,extended_filter=1,high_pass_filter=1,noise_suppression=1"

    # Load echo cancellation module
    echo "Setting up echo cancellation..."
    MODULE_ID=$(pactl load-module module-echo-cancel \
    aec_method=webrtc \
    source_master=$MIC_SOURCE \
    sink_master=$SPEAKER_SINK \
    source_name=echo-cancel-source \
    sink_name=echo-cancel-sink \
    channels=1 \
    rate=16000 \
    aec_args=analog_gain_control=0,digital_gain_control=1,extended_filter=1,high_pass_filter=1,noise_suppression=1)
    
    echo $MODULE_ID
    
    # Wait for devices to be created
    echo "Waiting for echo cancelled devices..."
    for i in {1..10}; do
        if pactl list sources | grep -q "echo-cancel-source"; then
            echo "Echo cancelled devices found"
            pactl set-default-source echo-cancel-source
            pactl set-default-sink echo-cancel-sink
            return 0
        fi
        sleep 1
    done

    # # Wait a moment for devices to be created
    # sleep 1

    # Check which suffix exists and set defaults accordingly
    echo "Setting echo cancelled devices as defaults..."
    if pactl list sources | grep -q "${MIC_SOURCE}.echo-cancel"; then
        echo "Using .echo-cancel suffix"
        pactl set-default-source "${MIC_SOURCE}.echo-cancel"
        pactl set-default-sink "${SPEAKER_SINK}.echo-cancel"
    elif pactl list sources | grep -q "${MIC_SOURCE}.2"; then
        echo "Using .2 suffix"
        pactl set-default-source "${MIC_SOURCE}.2"
        pactl set-default-sink "${SPEAKER_SINK}.2"
    else
        echo "Error: Could not find echo cancelled devices"
        return 1
    fi

    # Print final settings
    echo -e "\nFinal Audio Configuration:"
    echo "Default source: $(pactl get-default-source)"
    echo "Default sink: $(pactl get-default-sink)"
}

# Function to restore original audio settings
cleanup_audio() {
    echo "Cleaning up audio..."
    
    # Only try to restore if we have the initial values
    if [ -n "$INITIAL_SOURCE" ] && [ -n "$INITIAL_SINK" ]; then
        echo "Restoring to initial source: $INITIAL_SOURCE"
        echo "Restoring to initial sink: $INITIAL_SINK"
        
        # Kill pulseaudio gracefully
        pactl exit 2>/dev/null || true
        pulseaudio -k 2>/dev/null || true
        sleep 1  # Give it time to cleanup
        
        # Restart pulseaudio
        pulseaudio --start
        sleep 1  # Wait for it to start
        
        # Restore defaults only if pulseaudio is running
        if pulseaudio --check; then
            pactl set-default-source "$INITIAL_SOURCE" 2>/dev/null || true
            pactl set-default-sink "$INITIAL_SINK" 2>/dev/null || true
            echo "PulseAudio restored to initial state"
        else
            echo "Warning: PulseAudio not running, skipping restore"
        fi
    else
        echo "Warning: No initial audio state saved, skipping restore"
    fi
}

# Check for command line argument
if [ "$1" = "setup" ]; then
    setup_audio
elif [ "$1" = "cleanup" ]; then
    cleanup_audio
else
    echo "Usage: $0 [setup|cleanup]"
    exit 1
fi
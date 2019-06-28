import pyaudio
import wave

FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
CHUNK = 512
RECORD_SECONDS = 5
WAVE_OUTPUT_FILENAME = "recordedFile.wav"
device_index = 2
audio = pyaudio.PyAudio()

print("--------------------Input Deviece Lists : host 0---------------------")
info = audio.get_host_api_info_by_index(0)
numdevices = info.get('deviceCount')
for i in range(0, numdevices):
    channel = audio.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')
    if channel > 0:
        name = audio.get_device_info_by_host_api_device_index(0, i).get('name')
        rate = audio.get_device_info_by_host_api_device_index(0, i).get('defaultSampleRate')
        print "Input Device id [", i, "] : ", name, ", Rate : ", rate, ", Channel : ", channel

print("----------------------host 1---------------------")
info = audio.get_host_api_info_by_index(1)
numdevices = info.get('deviceCount')
for i in range(0, numdevices):
    name = audio.get_device_info_by_host_api_device_index(1, i).get('name')
    rate = audio.get_device_info_by_host_api_device_index(1, i).get('defaultSampleRate')
    channel = audio.get_device_info_by_host_api_device_index(1, i).get('maxInputChannels')
    print "Input Device id [", i, "] : ", name, ", Rate : ", rate, ", Channel : ", channel

print("-------------------------------------------------------------")

index = (input())
print("recording via index "+str(index))

stream = audio.open(format=FORMAT, channels=CHANNELS,
                rate=RATE, input=True,input_device_index = 0,
                frames_per_buffer=CHUNK)
print ("recording started")
Recordframes = []

for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    Recordframes.append(data)
print ("recording stopped")

stream.stop_stream()
stream.close()
audio.terminate()

waveFile = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
waveFile.setnchannels(CHANNELS)
waveFile.setsampwidth(audio.get_sample_size(FORMAT))
waveFile.setframerate(RATE)
waveFile.writeframes(b''.join(Recordframes))
waveFile.close()

#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2017 Google Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Google Cloud Speech API sample application using the streaming API.

NOTE: This module requires the additional dependency `pyaudio`. To install
using pip:

    pip install pyaudio

Example usage:
    python transcribe_streaming_mic.py
"""

# [START speech_transcribe_streaming_mic]
from __future__ import division

import re
import sys
import os
os.environ["GOOGLE_APPLICATION_CREDENTIALS"]='/home/mtec/catkin_ws/src/cobot/ros_speech2text/cobot-speech-077e4218112f.json'

from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
import pyaudio
from six.moves import queue

import rospy
import time
from std_msgs.msg import String

import wave
from ctypes import *

# Audio recording parameters
RATE = 44100
CHUNK = int(RATE / 10)  # 100ms

t = time.time()
node_name = ""

class MicrophoneStream(object):
    """Opens a recording stream as a generator yielding the audio chunks."""
    def __init__(self, rate, chunk):
        self._rate = rate
        self._chunk = chunk

        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()
        self._frames = []
        self.closed = True

    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()

        print("--------------------Input Deviece Lists : host 0---------------------")
        info = self._audio_interface.get_host_api_info_by_index(0)
        numdevices = info.get('deviceCount')
        for i in range(0, numdevices):
            channel = self._audio_interface.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')
            if channel > 0:
                name = self._audio_interface.get_device_info_by_host_api_device_index(0, i).get('name')
                rate = self._audio_interface.get_device_info_by_host_api_device_index(0, i).get('defaultSampleRate')
                print "Input Device id [", i, "] : ", name, ", Rate : ", rate, ", Channel : ", channel

        input_idx = rospy.get_param(rospy.get_name() + '/audio_device_idx', None)
#        input_idx = 0

        self._audio_stream = self._audio_interface.open(
            # The API currently only supports 1-channel (mono) audio
            # https://goo.gl/z757pE
            rate=self._rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            input_device_index=input_idx,
            frames_per_buffer=self._chunk,
            # Run the audio stream asynchronously to fill the buffer object.
            # This is necessary so that the input device's buffer doesn't
            # overflow while the calling thread makes network requests, etc.
            stream_callback=self._fill_buffer,
        )

        self.closed = False

        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)
        self._audio_interface.terminate()

        self.wf.writeframes(b''.join(self._frames))
        self.wf.close()

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        """Continuously collect data from the audio stream, into the buffer."""
        self._buff.put(in_data)
        self._frames.append(in_data)
        return None, pyaudio.paContinue

    def generator(self):
        self.wf = wave.open("cobot_speech2text.wav", 'wb')
        self.wf.setnchannels(1)
        self.wf.setsampwidth(self._audio_interface.get_sample_size(pyaudio.paInt16))
        self.wf.setframerate(RATE)
        while not self.closed and not rospy.is_shutdown():
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            # Now consume whatever other data's still buffered.
            while not rospy.is_shutdown():
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            # print("generator")
            yield b''.join(data)


def listen_print_loop(responses):
    rospy.loginfo("def listen_print_loop(responses):")

    num_chars_printed = 0
    for response in responses:
        rospy.logdebug("listen_print_loop")
        if rospy.is_shutdown():
            break

        if not response.results:
            continue

        result = response.results[0]
        if not result.alternatives:
            continue

        transcript = result.alternatives[0].transcript
        overwrite_chars = ' ' * (num_chars_printed - len(transcript))

        if not result.is_final:
            rospy.logdebug(transcript + overwrite_chars)
            num_chars_printed = len(transcript)

        else:
            rospy.logdebug(transcript + overwrite_chars)
            rospy.loginfo(transcript.encode('utf-8'))
            pub_text.publish(String(transcript))

            if re.search(r'\b(Exit|exit|Quit|quit)\b', transcript, re.I):
                rospy.loginfo('Exiting..')
                break
            num_chars_printed = 0

def main():
    # See http://g.co/cloud/speech/docs/languages
    # for a list of supported languages.
    language_code = 'th-TH'  # a BCP-47 language tag

    cobot_context = [ types.SpeechContext
                ( phrases=
                ['teach mode', 'normal mode', 'pickplace mode',
                 'บันทึกจุดที่', 'ไปจุดที่', 'กล่องที่', 'เริ่มได้', 'หยุด', 'go home',
                 'หนึ่ง', 'สอง', 'สาม',
                ],
                )
                ]
    client = speech.SpeechClient()
    config = types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code=language_code,
        speech_contexts=cobot_context
        )
    streaming_config = types.StreamingRecognitionConfig(
        config=config,
        interim_results=True)

    rospy.loginfo('cobot_speech2text')
    while not rospy.is_shutdown():
        try:
            rospy.loginfo('start')
            with MicrophoneStream(RATE, CHUNK) as stream:
                audio_generator = stream.generator()
                requests = (types.StreamingRecognizeRequest(audio_content=content)
                            for content in audio_generator)

                responses = client.streaming_recognize(streaming_config, requests)

                # Now, put the transcription responses to use.
                rospy.loginfo('response')
                listen_print_loop(responses)
        except:
            rospy.loginfo('end : %s' % str(sys.exc_info()[0]))

if __name__ == '__main__':
    rospy.init_node('cobot_speech2text', anonymous=True)
    rospy.loginfo('cobot_speech2text_starting...')
    global node_name
    node_name = rospy.get_name()
    rospy.loginfo(node_name)
    pub_text = rospy.Publisher('/cobot/speech/tts', String, queue_size=10)
    main()
# [END speech_transcribe_streaming_mic]

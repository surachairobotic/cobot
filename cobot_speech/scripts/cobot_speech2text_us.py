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

#########

import lib_speech

#########

homedir = os.path.expanduser("~")
os.environ["GOOGLE_APPLICATION_CREDENTIALS"]=homedir+('/catkin_ws/src/cobot/cobot_speech/cobot-speech-260419-fb5001458a83.json')

from google.cloud import speech_v1p1beta1
from google.cloud.speech_v1p1beta1 import enums
from google.cloud.speech_v1p1beta1 import types
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
node_name = String()

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
        idx = -1
        for i in range(0, numdevices):
            channel = self._audio_interface.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')
#            if channel > 0:
            name = self._audio_interface.get_device_info_by_host_api_device_index(0, i).get('name')
            rate = self._audio_interface.get_device_info_by_host_api_device_index(0, i).get('defaultSampleRate')
            print "Input Device id [", i, "] : ", name, ", Rate : ", rate, ", Channel : ", channel
            if name == 'default':
                idx = i

        if idx == -1:
            input_idx = rospy.get_param(rospy.get_name() + '/audio_device_idx', None)
        else:
            input_idx = idx
        rospy.loginfo("input_idx = %d" % input_idx)

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

        s = len(result.alternatives)
        for i in range(s):
            transcript = result.alternatives[i].transcript

            #########

            lib_speech.process_sentence(transcript.encode('utf-8'), pub_text)

            #########

            if not result.is_final:
                rospy.loginfo('US[%d/%d] : %s' % (i, s, transcript.encode('utf-8')))
            elif i is 0:
                rospy.logwarn('US[%d/%d] : %s' % (i, s, transcript.encode('utf-8')))
                # pub_text.publish(String(transcript))
            else:
                rospy.loginfo('US[%d/%d] : %s' % (i, s, transcript.encode('utf-8')))

            if re.search(r'\b(Exit|exit|Quit|quit)\b', transcript, re.I):
                rospy.loginfo('Exiting..')
                break
            num_chars_printed = 0

def main():
    # See http://g.co/cloud/speech/docs/languages
    # for a list of supported languages.
    language_code = 'en-US'  # a BCP-47 language tag

    cobot_phrases = ['learn mode', 'save point', 'pick box number one', 'pick box number two']
    # cobot_phrases = ['easy mode', 'jog mode', 'automatic mode', 'box', 'object', 'start','go home', 'go to point', 'save point number', 'one', 'two', 'three']

    #cobot_phrases = ['teach mode']
    cobot_boost = 20
    #boost=cobot_boost
    cobot_context = [ types.SpeechContext(phrases=cobot_phrases, boost=cobot_boost) ]
    cobot_meta = types.RecognitionMetadata(
        interaction_type='VOICE_COMMAND',
#        microphone_distance='NEARFIELD',
        original_media_type='AUDIO',
        recording_device_type='OTHER_INDOOR_DEVICE',
        audio_topic='Voice command to control robot arm'
        )
    client = speech_v1p1beta1.SpeechClient()
    config = types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code=language_code,
        max_alternatives=30,
        speech_contexts=cobot_context,
        enable_word_time_offsets=True,
        enable_word_confidence=True,
        enable_automatic_punctuation=True,
        model='command_and_search',
        use_enhanced=True,
        metadata=cobot_meta
        )
    streaming_config = types.StreamingRecognitionConfig(
        config=config,
        interim_results=True)

    # rospy.loginfo('cobot_speech2text')
    # while not rospy.is_shutdown():
    #     try:
    rospy.loginfo('start')
    with MicrophoneStream(RATE, CHUNK) as stream:
        audio_generator = stream.generator()
        requests = (types.StreamingRecognizeRequest(audio_content=content)
                    for content in audio_generator)

        responses = client.streaming_recognize(streaming_config, requests)

        # Now, put the transcription responses to use.
        rospy.loginfo('response')
        listen_print_loop(responses)
        # except:
        #     rospy.loginfo('end : %s' % str(sys.exc_info()[0]))

if __name__ == '__main__':
    rospy.init_node('cobot_speech2text_us', anonymous=True)
    rospy.loginfo('cobot_speech2text_starting...')
    global node_name
    node_name = rospy.get_name()
    rospy.loginfo(node_name)
    pub_text = rospy.Publisher('/cobot/speech/stt/en', String, queue_size=10)
    while not rospy.is_shutdown():
        try:
            main()
        except:
            rospy.loginfo('end : %s' % str(sys.exc_info()[0]))
# [END speech_transcribe_streaming_mic]

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

# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  # 100ms

t = time.time()

class MicrophoneStream(object):
    """Opens a recording stream as a generator yielding the audio chunks."""
    def __init__(self, rate, chunk):
        self._rate = rate
        self._chunk = chunk

        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()
        self.closed = True

    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            # The API currently only supports 1-channel (mono) audio
            # https://goo.gl/z757pE
            channels=1, rate=self._rate,
            input=True, frames_per_buffer=self._chunk,
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

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        """Continuously collect data from the audio stream, into the buffer."""
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def generator(self):
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b''.join(data)


def listen_print_loop(responses):
    rospy.loginfo("aaaaaaaaaaaaaa")

    num_chars_printed = 0
    plus_one = [[u'หนึ่ง','1'], [u'สอง','2'], [u'สาม','3'], [u'สี่',u'ห้า',u'หก',u'เจ็ด',u'แปด',u'เก้า',u'สิบ','4','5','6','7','8','9','10']]
    for response in responses:
        rospy.loginfo("listen_print_loop")
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
            print(transcript + overwrite_chars)
            num_chars_printed = len(transcript)

        else:
            i=0
            what_number = [0.0]*4
            for num in range(len(plus_one)):
              for k in range(len(plus_one[num])):
                if result.alternatives[i].transcript.find(plus_one[num][k]) != -1:
                  what_number[num] = what_number[num] + 1
                  break
            if result.alternatives[i].transcript.find(u'ึ') != -1:
              what_number[0] = what_number[0] + 0.01
            if result.alternatives[i].transcript.find(u'อ') != -1:
              what_number[1] = what_number[1] + 0.01
            if result.alternatives[i].transcript.find(u'า') != -1:
              what_number[2] = what_number[2] + 0.01

            print(transcript + overwrite_chars)
            print(what_number)

            if (what_number[0]+what_number[1]+what_number[2]) > 0:
              b_other = True
              for i in range(len(what_number)-1):
                if what_number[3] < what_number[i]:
                  b_other = False
                  break
              cmd = '~/catkin_ws/src/cobot/ros_speech2text/speech.sh '
              if not b_other:
                if what_number[0] > what_number[1] and what_number[0] > what_number[2]:
                  c = 'กล่องที่หนึ่ง'
                  rospy.loginfo('Number One');
                  pub_text.publish(String('M1'))
                elif what_number[1] > what_number[0] and what_number[1] > what_number[2]:
                  c = 'กล่องที่สอง'
                  rospy.loginfo('Number Two');
                  pub_text.publish(String('M2'))
                elif what_number[2] > what_number[0] and what_number[2] > what_number[1]:
                  c = 'กล่องที่สาม'
                  rospy.loginfo('Number Three');
                  pub_text.publish(String('M3'))
              else:
                c = 'อะไรนะคะ'
                rospy.loginfo('Other');
              os.system(cmd+c)

            if re.search(r'\b(Exit|exit|Quit|quit|xxxx)\b', transcript, re.I):
                print('Exiting..')
                break
            num_chars_printed = 0

def main():
    # See http://g.co/cloud/speech/docs/languages
    # for a list of supported languages.
    language_code = 'th-TH'  # a BCP-47 language tag

    client = speech.SpeechClient()
    config = types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code=language_code
        )
    streaming_config = types.StreamingRecognitionConfig(
        config=config,
        interim_results=True)

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
    rospy.init_node('cobot_speech', anonymous=True)
    rospy.loginfo('cobot_speech_starting...')
    pub_text = rospy.Publisher('/cobot/speech/text', String, queue_size=10)
    main()
# [END speech_transcribe_streaming_mic]

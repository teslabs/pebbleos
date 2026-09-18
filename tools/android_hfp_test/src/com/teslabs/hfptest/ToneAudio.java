/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
package com.teslabs.hfptest;

import android.media.AudioAttributes;
import android.media.AudioDeviceInfo;
import android.media.AudioFormat;
import android.media.AudioRecord;
import android.media.AudioTrack;
import android.media.MediaRecorder;
import android.util.Log;

final class ToneAudio {
  private volatile boolean running;
  private Thread playback, capture;
  synchronized void start() {
    if (running)
      return;
    running = true;
    playback = new Thread(this::play, "HfpTestPlayback");
    capture = new Thread(this::record, "HfpTestCapture");
    playback.start();
    capture.start();
  }
  synchronized void stop() {
    running = false;
  }
  private static String device(AudioDeviceInfo device) {
    return device == null ? "none" : device.getType() + ":" + device.getProductName();
  }
  private void play() {
    AudioTrack track = null;
    try {
      int rate = 16000;
      int size = Math.max(3200,
          AudioTrack.getMinBufferSize(rate, AudioFormat.CHANNEL_OUT_MONO,
              AudioFormat.ENCODING_PCM_16BIT));
      track = new AudioTrack.Builder()
                  .setAudioAttributes(new AudioAttributes.Builder()
                          .setUsage(AudioAttributes.USAGE_VOICE_COMMUNICATION)
                          .setContentType(AudioAttributes.CONTENT_TYPE_SPEECH)
                          .build())
                  .setAudioFormat(new AudioFormat.Builder()
                          .setSampleRate(rate)
                          .setChannelMask(AudioFormat.CHANNEL_OUT_MONO)
                          .setEncoding(AudioFormat.ENCODING_PCM_16BIT)
                          .build())
                  .setBufferSizeInBytes(size)
                  .build();
      short[] samples = new short[320];
      for (int i = 0; i < samples.length; ++i)
        samples[i] = (short) (4000 * Math.sin(2 * Math.PI * 1000 * i / rate));
      track.play();
      long next = 0, count = 0;
      while (running) {
        int n = track.write(samples, 0, samples.length);
        if (n < 0)
          throw new IllegalStateException("AudioTrack write " + n);
        count += n;
        if (System.nanoTime() > next) {
          Log.i("HfpTest",
              "TX samples=" + count + " route=" + device(track.getRoutedDevice())
                  + " underruns=" + track.getUnderrunCount());
          next = System.nanoTime() + 1000000000L;
        }
      }
    } catch (Exception e) {
      Log.e("HfpTest", "Playback failed", e);
    } finally {
      if (track != null) {
        try {
          track.stop();
        } finally {
          track.release();
        }
      }
    }
  }
  private void record() {
    AudioRecord recorder = null;
    try {
      int size = Math.max(3200,
          AudioRecord.getMinBufferSize(16000, AudioFormat.CHANNEL_IN_MONO,
              AudioFormat.ENCODING_PCM_16BIT));
      recorder = new AudioRecord(MediaRecorder.AudioSource.VOICE_COMMUNICATION, 16000,
          AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT, size);
      short[] samples = new short[320];
      recorder.startRecording();
      long next = 0, count = 0;
      while (running) {
        int n = recorder.read(samples, 0, samples.length);
        if (n < 0)
          throw new IllegalStateException("AudioRecord read " + n);
        count += n;
        double energy = 0;
        for (int i = 0; i < n; ++i)
          energy += (double) samples[i] * samples[i];
        if (System.nanoTime() > next) {
          Log.i("HfpTest",
              "RX samples=" + count + " route=" + device(recorder.getRoutedDevice())
                  + " rms=" + Math.sqrt(energy / Math.max(1, n)));
          next = System.nanoTime() + 1000000000L;
        }
      }
    } catch (Exception e) {
      Log.e("HfpTest", "Capture failed", e);
    } finally {
      if (recorder != null) {
        try {
          recorder.stop();
        } finally {
          recorder.release();
        }
      }
    }
  }
}

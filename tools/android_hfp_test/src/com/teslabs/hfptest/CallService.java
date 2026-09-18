/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
package com.teslabs.hfptest;

import android.bluetooth.BluetoothManager;
import android.os.Handler;
import android.os.Looper;
import android.net.Uri;
import android.telecom.CallAudioState;
import android.telecom.Connection;
import android.telecom.ConnectionRequest;
import android.telecom.ConnectionService;
import android.telecom.DisconnectCause;
import android.telecom.PhoneAccountHandle;
import android.telecom.TelecomManager;
import android.util.Log;

public class CallService extends ConnectionService {
  static volatile TestConnection current;
  static String deviceAddress;

  public class TestConnection extends Connection {
    private final ToneAudio audio = new ToneAudio();
    private final Handler handler = new Handler(Looper.getMainLooper());
    private final Runnable timeout = () -> finish(DisconnectCause.LOCAL);
    private boolean finished;
    TestConnection(boolean incoming) {
      handler.postDelayed(timeout, 300000);
      setConnectionProperties(PROPERTY_SELF_MANAGED);
      setAudioModeIsVoip(true);
      // Display-only reserved test number; the local account still routes via SIP.
      setAddress(Uri.fromParts("tel", "+12025550100", null), TelecomManager.PRESENTATION_ALLOWED);
      setCallerDisplayName("Local HFP test", TelecomManager.PRESENTATION_ALLOWED);
      if (incoming)
        setRinging();
      else
        setDialing();
      Log.i("HfpTest", incoming ? "RINGING" : "DIALING");
    }
    @Override
    public void onShowIncomingCallUi() {
      Log.i("HfpTest", "Incoming call ready");
    }
    @Override
    public void onAnswer() {
      if (finished || getState() == STATE_ACTIVE)
        return;
      setActive();
      audio.start();
      route();
      Log.i("HfpTest", "ACTIVE");
    }
    @Override
    public void onAnswer(int videoState) {
      onAnswer();
    }
    @Override
    public void onReject() {
      finish(DisconnectCause.REJECTED);
    }
    @Override
    public void onDisconnect() {
      finish(DisconnectCause.LOCAL);
    }
    @Override
    public void onAbort() {
      finish(DisconnectCause.CANCELED);
    }
    @Override
    public void onCallAudioStateChanged(CallAudioState state) {
      Log.i("HfpTest", "Audio " + state);
    }
    void route() {
      if (deviceAddress == null)
        return;
      requestBluetoothAudio(
          getSystemService(BluetoothManager.class).getAdapter().getRemoteDevice(deviceAddress));
    }
    private void finish(int cause) {
      if (finished)
        return;
      finished = true;
      handler.removeCallbacks(timeout);
      audio.stop();
      setDisconnected(new DisconnectCause(cause));
      destroy();
      if (current == this)
        current = null;
      Log.i("HfpTest", "DISCONNECTED cause=" + cause);
    }
  }

  private Connection create(boolean incoming) {
    if (current != null)
      return Connection.createFailedConnection(new DisconnectCause(DisconnectCause.BUSY));
    current = new TestConnection(incoming);
    return current;
  }
  @Override
  public Connection onCreateIncomingConnection(PhoneAccountHandle manager,
      ConnectionRequest request) {
    return create(true);
  }
  @Override
  public Connection onCreateOutgoingConnection(PhoneAccountHandle manager,
      ConnectionRequest request) {
    return create(false);
  }
  @Override
  public void onCreateIncomingConnectionFailed(PhoneAccountHandle manager,
      ConnectionRequest request) {
    Log.e("HfpTest", "Incoming connection rejected by Telecom");
  }
  @Override
  public void onCreateOutgoingConnectionFailed(PhoneAccountHandle manager,
      ConnectionRequest request) {
    Log.e("HfpTest", "Outgoing connection rejected by Telecom");
  }
}

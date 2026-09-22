/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */
package com.teslabs.hfptest;

import android.app.Activity;
import android.content.ComponentName;
import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.telecom.PhoneAccount;
import android.telecom.PhoneAccountHandle;
import android.telecom.TelecomManager;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;

public class MainActivity extends Activity {
  static PhoneAccountHandle account;
  static final Uri ADDRESS = Uri.parse("sip:loopback@hfp.invalid");

  @Override
  public void onCreate(Bundle state) {
    super.onCreate(state);
    getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
    TelecomManager telecom = getSystemService(TelecomManager.class);
    account = new PhoneAccountHandle(new ComponentName(this, CallService.class), "local-test");
    telecom.registerPhoneAccount(PhoneAccount.builder(account, "Local HFP test")
            .setCapabilities(PhoneAccount.CAPABILITY_SELF_MANAGED)
            .addSupportedUriScheme(PhoneAccount.SCHEME_SIP)
            .build());
    LinearLayout layout = new LinearLayout(this);
    layout.setOrientation(LinearLayout.VERTICAL);
    layout.setPadding(32, 64, 32, 32);
    TextView description = new TextView(this);
    description.setText(
        "Local HFP test\nCalls stay on this phone.\nActive calls play a 1 kHz tone and measure microphone level. No audio is saved.\nDiagnostics: adb logcat -s HfpTest");
    layout.addView(description);
    for (String action : new String[] {"incoming", "outgoing", "active", "bluetooth", "hangup"}) {
      Button button = new Button(this);
      button.setText(action);
      button.setOnClickListener(v -> command(action));
      layout.addView(button);
    }
    setContentView(layout);
    onNewIntent(getIntent());
  }

  @Override
  public void onNewIntent(Intent intent) {
    super.onNewIntent(intent);
    String address = intent.getStringExtra("device");
    if (address != null)
      CallService.deviceAddress = address;
    String action = intent.getStringExtra("command");
    if (action != null)
      command(action);
  }

  private void command(String action) {
    TelecomManager telecom = getSystemService(TelecomManager.class);
    Bundle extras = new Bundle();
    switch (action) {
      case "incoming":
        if (CallService.current != null)
          return;
        extras.putParcelable(TelecomManager.EXTRA_INCOMING_CALL_ADDRESS, ADDRESS);
        telecom.addNewIncomingCall(account, extras);
        break;
      case "outgoing":
        if (CallService.current != null)
          return;
        extras.putParcelable(TelecomManager.EXTRA_PHONE_ACCOUNT_HANDLE, account);
        telecom.placeCall(ADDRESS, extras);
        break;
      case "active":
        if (CallService.current != null)
          CallService.current.onAnswer();
        break;
      case "bluetooth":
        if (CallService.current != null)
          CallService.current.route();
        break;
      case "hangup":
        if (CallService.current != null)
          CallService.current.onDisconnect();
        break;
      default:
        throw new IllegalArgumentException(action);
    }
  }
}

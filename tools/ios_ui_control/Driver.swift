// SPDX-FileCopyrightText: 2026 Core Devices LLC
// SPDX-License-Identifier: Apache-2.0
import AVFoundation
import CallKit
import SwiftUI

// Local CallKit events and a quiet tone; no cellular, SIP, or network connection.
final class LocalCall: NSObject, ObservableObject, CXProviderDelegate {
  @Published var state = "Idle"
  @Published var route = "No audio"
  @Published var capture = "No samples"
  @Published var busy = false
  @Published var events = ""
  private let provider: CXProvider
  private let controller = CXCallController()
  private var call: UUID?
  private var expiry: Timer?
  private var engine: AVAudioEngine?
  private var player: AVAudioPlayerNode?
  private var engineObserver: NSObjectProtocol?
  private var routeObserver: NSObjectProtocol?
  private var restart: DispatchWorkItem?
  private var audioActive = false
  private var generation = 0

  override init() {
    let config = CXProviderConfiguration()
    config.supportedHandleTypes = [.phoneNumber]
    config.supportsVideo = false
    config.maximumCallGroups = 1
    config.maximumCallsPerCallGroup = 1
    config.includesCallsInRecents = false
    provider = CXProvider(configuration: config)
    super.init()
    provider.setDelegate(self, queue: .main)
    routeObserver = NotificationCenter.default.addObserver(
      forName: AVAudioSession.routeChangeNotification, object: nil, queue: .main
    ) { [weak self] _ in self?.updateRoute() }
  }

  deinit {
    if let routeObserver { NotificationCenter.default.removeObserver(routeObserver) }
  }

  private func updateRoute() {
    guard audioActive else { return }
    let session = AVAudioSession.sharedInstance()
    let input = session.currentRoute.inputs.map { $0.portType.rawValue }.joined(separator: ", ")
    let output = session.currentRoute.outputs.map { $0.portType.rawValue }.joined(separator: ", ")
    route = "Input: \(input)\nOutput: \(output)\n\(Int(session.sampleRate)) Hz"
  }

  private func note(_ text: String) {
    events = (events.split(separator: "\n").suffix(4).map(String.init) + [text]).joined(
      separator: "\n")
    print("HFP_TEST \(text)")
  }

  func providerDidBegin(_ provider: CXProvider) { note("Provider ready") }

  func prepare(_ incoming: Bool) {
    note(incoming ? "Request incoming" : "Request outgoing")
    guard call == nil else { return }
    AVAudioApplication.requestRecordPermission { [weak self] granted in
      DispatchQueue.main.async {
        guard let self else { return }
        self.note("Microphone allowed=\(granted)")
        if granted { self.begin(incoming) } else { self.state = "Microphone permission required" }
      }
    }
  }

  private func configureAudio() throws {
    let session = AVAudioSession.sharedInstance()
    try session.setCategory(.playAndRecord, mode: .voiceChat, options: [.allowBluetoothHFP])
    try session.setPreferredSampleRate(16000)
  }

  private func begin(_ incoming: Bool) {
    guard call == nil else { return }
    note("Creating local call")
    let id = UUID()
    call = id
    busy = true
    state = incoming ? "Incoming" : "Outgoing"
    capture = "No samples"
    expiry = Timer.scheduledTimer(withTimeInterval: 180, repeats: false) { [weak self] _ in
      guard let self, self.call == id else { return }
      self.provider.reportCall(with: id, endedAt: Date(), reason: .remoteEnded)
      self.finish()
    }
    let update = CXCallUpdate()
    update.remoteHandle = CXHandle(type: .phoneNumber, value: "+12025550100")
    update.localizedCallerName = "Local HFP test"
    update.supportsHolding = false
    update.supportsGrouping = false
    update.supportsUngrouping = false
    update.supportsDTMF = false
    if incoming {
      provider.reportNewIncomingCall(with: id, update: update) { [weak self] error in
        DispatchQueue.main.async {
          guard let self, self.call == id, let error else { return }
          self.note("Incoming failed: \(error)")
          self.finish()
          self.state = error.localizedDescription
        }
      }
    } else {
      controller.request(
        CXTransaction(action: CXStartCallAction(call: id, handle: update.remoteHandle!))
      ) { [weak self] error in
        DispatchQueue.main.async {
          guard let self, self.call == id else { return }
          if let error {
            self.finish()
            self.state = error.localizedDescription
          } else {
            self.provider.reportCall(with: id, updated: update)
          }
        }
      }
    }
  }

  func end() {
    guard let id = call else { return }
    controller.request(CXTransaction(action: CXEndCallAction(call: id))) { [weak self] error in
      DispatchQueue.main.async {
        guard let self, self.call == id, let error else { return }
        self.state = error.localizedDescription
      }
    }
  }

  func useWatch() {
    let session = AVAudioSession.sharedInstance()
    let inputs = session.availableInputs?.filter { $0.portType == .bluetoothHFP } ?? []
    guard inputs.count == 1 else {
      state = "Expected one HFP input, found \(inputs.count)"
      return
    }
    do { try session.setPreferredInput(inputs[0]) } catch { state = error.localizedDescription }
  }

  func providerDidReset(_ provider: CXProvider) {
    note("Provider reset")
    finish()
  }

  func provider(_ provider: CXProvider, timedOutPerforming action: CXAction) {
    guard let action = action as? CXCallAction, action.callUUID == call else { return }
    provider.reportCall(with: action.callUUID, endedAt: Date(), reason: .failed)
    finish()
    state = "CallKit action timed out"
  }

  func provider(_ provider: CXProvider, perform action: CXAnswerCallAction) {
    note("Answer action")
    guard action.callUUID == call else {
      action.fail()
      return
    }
    do {
      try configureAudio()
      state = "Active"
      action.fulfill()
    } catch {
      action.fail()
      provider.reportCall(with: action.callUUID, endedAt: Date(), reason: .failed)
      finish()
      state = error.localizedDescription
    }
  }

  func provider(_ provider: CXProvider, perform action: CXStartCallAction) {
    guard action.callUUID == call else {
      action.fail()
      return
    }
    do {
      try configureAudio()
      provider.reportOutgoingCall(with: action.callUUID, startedConnectingAt: Date())
      action.fulfill()
      DispatchQueue.main.asyncAfter(deadline: .now() + 1) { [weak self] in
        guard let self, self.call == action.callUUID else { return }
        provider.reportOutgoingCall(with: action.callUUID, connectedAt: Date())
        self.state = "Active"
      }
    } catch {
      action.fail()
      finish()
      state = error.localizedDescription
    }
  }

  func provider(_ provider: CXProvider, perform action: CXEndCallAction) {
    note("End action")
    guard action.callUUID == call else {
      action.fail()
      return
    }
    finish()
    action.fulfill()
  }

  func provider(_ provider: CXProvider, didActivate audioSession: AVAudioSession) {
    note("Audio activated")
    guard call != nil else { return }
    audioActive = true
    updateRoute()
    startAudio()
  }

  func provider(_ provider: CXProvider, didDeactivate audioSession: AVAudioSession) {
    audioActive = false
    stopAudio()
    route = "Audio inactive"
  }

  private func finish() {
    call = nil
    busy = false
    expiry?.invalidate()
    expiry = nil
    audioActive = false
    stopAudio()
    state = "Idle"
    route = "No audio"
  }

  private func stopAudio() {
    generation += 1
    restart?.cancel()
    restart = nil
    if let engineObserver { NotificationCenter.default.removeObserver(engineObserver) }
    engineObserver = nil
    engine?.stop()
    player?.stop()
    engine = nil
    player = nil
  }

  private func startAudio() {
    stopAudio()
    guard call != nil, audioActive else { return }
    let engine = AVAudioEngine()
    let player = AVAudioPlayerNode()
    let format = AVAudioFormat(standardFormatWithSampleRate: 16000, channels: 1)!
    let buffer = AVAudioPCMBuffer(pcmFormat: format, frameCapacity: 16000)!
    buffer.frameLength = 16000
    for i in 0..<16000 {
      buffer.floatChannelData![0][i] = 0.03 * sin(Float(i) * 2 * .pi * 440 / 16000)
    }
    engine.attach(player)
    engine.connect(player, to: engine.mainMixerNode, format: format)
    let inputFormat = engine.inputNode.outputFormat(forBus: 0)
    guard inputFormat.channelCount > 0, inputFormat.sampleRate > 0 else {
      state = "Audio input unavailable"
      return
    }
    let epoch = generation
    var samples: UInt64 = 0
    var intervalSamples: UInt64 = 0
    var energy: Double = 0
    engine.inputNode.installTap(onBus: 0, bufferSize: 1024, format: inputFormat) {
      [weak self] pcm, _ in
      guard let channel = pcm.floatChannelData?[0] else { return }
      for i in 0..<Int(pcm.frameLength) { energy += Double(channel[i]) * Double(channel[i]) }
      samples += UInt64(pcm.frameLength)
      intervalSamples += UInt64(pcm.frameLength)
      if intervalSamples >= UInt64(inputFormat.sampleRate) {
        let rms = sqrt(energy / Double(intervalSamples))
        let count = samples
        energy = 0
        intervalSamples = 0
        DispatchQueue.main.async {
          guard let self, self.generation == epoch else { return }
          self.capture = String(format: "RX samples=%llu rms=%.6f", count, rms)
          self.updateRoute()
        }
      }
    }
    self.engine = engine
    self.player = player
    engineObserver = NotificationCenter.default.addObserver(
      forName: .AVAudioEngineConfigurationChange, object: engine, queue: .main
    ) { [weak self] _ in
      guard let self, self.audioActive else { return }
      self.restart?.cancel()
      let work = DispatchWorkItem { [weak self] in self?.startAudio() }
      self.restart = work
      DispatchQueue.main.asyncAfter(deadline: .now() + 0.3, execute: work)
    }
    do {
      try engine.start()
      player.scheduleBuffer(buffer, at: nil, options: .loops)
      player.play()
    } catch {
      stopAudio()
      state = error.localizedDescription
    }
  }
}

@main struct Driver: App {
  @StateObject private var call = LocalCall()
  var body: some Scene {
    WindowGroup {
      VStack(spacing: 20) {
        Text("Local HFP test").font(.title)
        Text("Local calls only. No number is dialed.\nCalls end after three minutes.")
        Text(call.state).accessibilityIdentifier("call-state")
        Text(call.route).accessibilityIdentifier("audio-route")
        Text(call.capture).accessibilityIdentifier("capture-status")
        Text(call.events).font(.caption).accessibilityIdentifier("call-events")
        Button("Incoming") { call.prepare(true) }.disabled(call.busy)
        Button("Outgoing") { call.prepare(false) }.disabled(call.busy)
        Button("Use watch") { call.useWatch() }.disabled(!call.busy)
        Button("End call") { call.end() }.disabled(!call.busy)
      }.padding()
    }
  }
}

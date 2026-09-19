// SPDX-FileCopyrightText: 2026 Core Devices LLC
// SPDX-License-Identifier: Apache-2.0
import XCTest

final class Control: XCTestCase {
  func testControl() throws {
    continueAfterFailure = false
    executionTimeAllowance = 3600
    let directory = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
    let commandURL = directory.appendingPathComponent("command.json")
    let responseURL = directory.appendingPathComponent("response.json")
    try? FileManager.default.removeItem(at: commandURL)
    var app = XCUIApplication(bundleIdentifier: "coredevices.coreapp")
    app.activate()
    func snapshot() throws {
      try app.debugDescription.write(
        to: directory.appendingPathComponent("tree.txt"), atomically: true, encoding: .utf8)
      try XCUIScreen.main.screenshot().pngRepresentation.write(
        to: directory.appendingPathComponent("screen.png"), options: .atomic)
    }
    try snapshot()
    print("HFP_CONTROL_READY \(directory.path)")
    let deadline = Date().addingTimeInterval(1800)
    var lastID = ""
    while Date() < deadline {
      RunLoop.current.run(until: Date().addingTimeInterval(0.25))
      guard let data = try? Data(contentsOf: commandURL),
        let cmd = (try? JSONSerialization.jsonObject(with: data)) as? [String: Any],
        let id = cmd["id"] as? String, id != lastID
      else { continue }
      lastID = id
      let action = cmd["action"] as? String ?? "snapshot"
      var result = "ok"
      if action == "stop" { break }
      app = XCUIApplication(bundleIdentifier: cmd["bundle"] as? String ?? "coredevices.coreapp")
      switch action {
      case "activate": app.activate()
      case "tap":
        let label = cmd["label"] as? String ?? ""
        let query = app.descendants(matching: .any).matching(
          NSPredicate(format: "label == %@ OR identifier == %@", label, label))
        let buttons = app.buttons.matching(
          NSPredicate(format: "label == %@ OR identifier == %@", label, label))
        let buttonMatches = buttons.allElementsBoundByIndex.filter { $0.isHittable }
        let matches =
          buttonMatches.isEmpty
          ? query.allElementsBoundByIndex.filter { $0.isHittable } : buttonMatches
        if matches.count == 1 {
          matches[0].tap()
        } else {
          result = "Expected one hittable match, found \(matches.count)"
        }
      case "coordinate":
        if let x = cmd["x"] as? Double, let y = cmd["y"] as? Double, x >= 0, x <= 1, y >= 0, y <= 1
        {
          app.coordinate(withNormalizedOffset: CGVector(dx: x, dy: y)).tap()
        } else {
          result = "Invalid coordinates"
        }
      case "swipe":
        switch cmd["direction"] as? String {
        case "up": app.swipeUp()
        case "down": app.swipeDown()
        case "left": app.swipeLeft()
        case "right": app.swipeRight()
        default: result = "Invalid direction"
        }
      case "snapshot": break
      default: result = "Unknown action"
      }
      RunLoop.current.run(until: Date().addingTimeInterval(0.5))
      try snapshot()
      let response: [String: Any] = ["id": id, "result": result]
      try JSONSerialization.data(withJSONObject: response).write(to: responseURL, options: .atomic)
      print("HFP_CONTROL \(id): \(result)")
    }
  }
}

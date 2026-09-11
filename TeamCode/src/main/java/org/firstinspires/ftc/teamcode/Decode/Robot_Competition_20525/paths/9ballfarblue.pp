{
  "startPoint": {
    "x": 63.5,
    "y": 8,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-ejdcksa5xy",
      "name": "Shoot",
      "endPoint": {
        "x": 61.60747663551402,
        "y": 17.38317757009345,
        "heading": "linear",
        "startDeg": 90,
        "endDeg": 115
      },
      "controlPoints": [],
      "color": "#57BAA9",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkcujzp1-c6x5sv",
      "name": "prepareIntake1",
      "endPoint": {
        "x": 41.19626168224299,
        "y": 36.0467289719626,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#955B95",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkcul74c-nkrb2w",
      "name": "Intake1",
      "endPoint": {
        "x": 7.50467289719626,
        "y": 36.0467289719626,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#867A96",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkcum79h-ng3iuy",
      "name": "Shoot2",
      "endPoint": {
        "x": 61.60747663551402,
        "y": 17.38317757009345,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 115
      },
      "controlPoints": [],
      "color": "#5D8C6B",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkcuo930-v0pa2l",
      "name": "Intake2",
      "endPoint": {
        "x": 18.121495327102803,
        "y": 13.33644859813084,
        "heading": "linear",
        "reverse": false,
        "startDeg": 115,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#76A96A",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkcvmdfk-h9eq81",
      "name": "Path 6",
      "endPoint": {
        "x": 61.60747663551402,
        "y": 17.38317757009345,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 115
      },
      "controlPoints": [],
      "color": "#B9859C",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-ejdcksa5xy"
    },
    {
      "kind": "wait",
      "id": "mkcuis0f-srsh0b",
      "name": "Wait",
      "durationMs": 3500,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "mkcujzp1-c6x5sv"
    },
    {
      "kind": "path",
      "lineId": "mkcul74c-nkrb2w"
    },
    {
      "kind": "path",
      "lineId": "mkcum79h-ng3iuy"
    },
    {
      "kind": "wait",
      "id": "mkcun300-mlixk8",
      "name": "Wait",
      "durationMs": 3500,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "mkcuo930-v0pa2l"
    },
    {
      "kind": "wait",
      "id": "mkcupctp-m58lkp",
      "name": "Wait",
      "durationMs": 2200,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "mkcvmdfk-h9eq81"
    },
    {
      "kind": "wait",
      "id": "mkcvmuwi-vxp9n5",
      "name": "Wait",
      "durationMs": 3500,
      "locked": false
    }
  ],
  "version": "1.2.1",
  "timestamp": "2026-01-13T17:41:23.923Z"
}
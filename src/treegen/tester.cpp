#include "BTBuilder.h"
#include "json.hpp"
#include <iostream>

int main(int argc, char** argv) {
    BTBuilder builder;
    std::cout << builder.build(R"###({
    "title": "A Behavior Tree",
    "description": "",
    "root": "87bf7cd5-56a3-4baf-a6c1-6993a9385ec3",
    "display": {
        "camera_x": 927.5,
        "camera_y": 492.5,
        "camera_z": 1,
        "x": -480,
        "y": 32
    },
    "properties": {
        "prop": "propval"
    },
    "nodes": {
        "b2ca3134-b75c-40f5-bb12-f28de905643e": {
            "id": "b2ca3134-b75c-40f5-bb12-f28de905643e",
            "name": "Sequence",
            "title": "Sequence",
            "description": "",
            "display": {
                "x": -832,
                "y": -320
            },
            "parameters": {},
            "properties": {},
            "children": []
        },
        "16dbe79a-bf4b-4b59-90cc-0e770cc99b20": {
            "id": "16dbe79a-bf4b-4b59-90cc-0e770cc99b20",
            "name": "Sequence",
            "title": "Sequence",
            "description": "",
            "display": {
                "x": -832,
                "y": -320
            },
            "parameters": {},
            "properties": {},
            "children": []
        },
        "87bf7cd5-56a3-4baf-a6c1-6993a9385ec3": {
            "id": "87bf7cd5-56a3-4baf-a6c1-6993a9385ec3",
            "name": "Priority",
            "title": "root",
            "description": "",
            "display": {
                "x": -320,
                "y": 32
            },
            "parameters": {},
            "properties": {},
            "children": [
                "94a5c370-6b1f-4e9d-862b-22371154b9e7",
                "d0dccdef-f194-4587-8e19-360fa859de42",
                "a3358a62-a1c7-4bb4-8089-aae0ac5bf492"
            ]
        },
        "d59a09b8-13e1-426d-8e57-222a61e2fabf": {
            "id": "d59a09b8-13e1-426d-8e57-222a61e2fabf",
            "name": "Runner",
            "title": "Runner",
            "description": "",
            "display": {
                "x": 48,
                "y": -96
            },
            "parameters": {},
            "properties": {
                "rprop": "rpropval"
            }
        },
        "d0dccdef-f194-4587-8e19-360fa859de42": {
            "id": "d0dccdef-f194-4587-8e19-360fa859de42",
            "name": "Error",
            "title": "Error",
            "description": "",
            "display": {
                "x": 48,
                "y": 0
            },
            "parameters": {},
            "properties": {}
        },
        "94a5c370-6b1f-4e9d-862b-22371154b9e7": {
            "id": "94a5c370-6b1f-4e9d-862b-22371154b9e7",
            "name": "Inverter",
            "title": "Inverter",
            "description": "",
            "display": {
                "x": -176,
                "y": -80
            },
            "parameters": {},
            "properties": {},
            "child": "d59a09b8-13e1-426d-8e57-222a61e2fabf"
        },
        "a3358a62-a1c7-4bb4-8089-aae0ac5bf492": {
            "id": "a3358a62-a1c7-4bb4-8089-aae0ac5bf492",
            "name": "MemPriority",
            "title": "prio",
            "description": "",
            "display": {
                "x": -176,
                "y": 112
            },
            "parameters": {},
            "properties": {},
            "children": [
                "73cc7db6-12d6-4504-84cf-1290edf20ee9"
            ]
        },
        "73cc7db6-12d6-4504-84cf-1290edf20ee9": {
            "id": "73cc7db6-12d6-4504-84cf-1290edf20ee9",
            "name": "RepeatUntilFailure",
            "title": "ruf",
            "description": "",
            "display": {
                "x": -16,
                "y": 112
            },
            "parameters": {
                "maxLoop": -1
            },
            "properties": {},
            "child": "77802e84-a123-448b-844b-dd95ada953c0"
        },
        "1fb09c63-490f-4e26-9d68-f4202bb5df14": {
            "id": "1fb09c63-490f-4e26-9d68-f4202bb5df14",
            "name": "Test",
            "title": "Test",
            "description": "",
            "display": {
                "x": -816,
                "y": 176
            },
            "parameters": {},
            "properties": {}
        },
        "77802e84-a123-448b-844b-dd95ada953c0": {
            "id": "77802e84-a123-448b-844b-dd95ada953c0",
            "name": "Test",
            "title": "Test",
            "description": "",
            "display": {
                "x": 272,
                "y": 112
            },
            "parameters": {},
            "properties": {}
        }
    },
    "custom_nodes": [
        {
            "name": "Test",
            "title": "Test",
            "category": "action"
        }
    ]
}
)###"_json);
}

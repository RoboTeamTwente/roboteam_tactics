#!/usr/bin/env python3
from __future__ import print_function

import subprocess
import sys
import os
import re
import argparse
import sys

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

def removeFirstSpaceIfPresent(x):
    if len(x) > 0:
        if x[0] == " ":
            return x[1:]

    return x

def processRawCommentsToString(comments):
    truncated = comments.split("\n")

    # Removes anything up until the first star in every line
    truncated = [x.split("*", 1)[-1] for x in truncated]
    # Removes first space if present
    truncated = [removeFirstSpaceIfPresent(x) for x in truncated]
    # Removes empty lines
    truncated = [x for x in truncated if len(x) > 0]

    return "\n".join(truncated)

def getDocumentation(docTarget, tacticsRoot):
    found = False
    foundAt = ""
    foundType = ""

    tacticsInclude = os.path.join(tacticsRoot, "include", "roboteam_tactics")

    locations = {
        "skills": os.path.join(tacticsInclude, "skills"),
        "conditions": os.path.join(tacticsInclude, "conditions"),
        "tactics": os.path.join(tacticsInclude, "tactics")
    }

    for locationType, location in locations.items():
        for entry in os.listdir(location):
            if not os.path.isdir(entry) and entry.endswith(".h"):
                entryWithoutExtension = entry[:-2]

                if entryWithoutExtension == docTarget:
                    foundAt = os.path.join(location, entry)
                    found = True
                    foundType = locationType
                    break

    if not found:
        # eprint("Could not find skill with name \"" + docTarget + "\"")
        # sys.exit(1)
        return None

    with open(foundAt, 'r') as f:
        contents = f.read();

    m = re.search("/\*.*?\*/.*/\*(.*)\*/", contents, re.DOTALL)

    if m:
        return processRawCommentsToString(m.group(1))
        # truncated = rawDocs.split("\n")

        # # Removes anything up until the first star in every line
        # truncated = [x.split("*", 1)[-1] for x in truncated]
        # # Removes first space if present
        # truncated = [removeFirstSpaceIfPresent(x) for x in truncated]
        # # Removes empty lines
        # truncated = [x for x in truncated if len(x) > 0]

        # return "\n".join(truncated)

    m = re.search("/\*(.*)\*/", contents, re.DOTALL)

    if m:
        return processRawCommentsToString(m.group(1))

    return None

def getAndPrintDocumentation(docTarget, roboteam_tactics_root):
    result = getDocumentation(docTarget, roboteam_tactics_root)

    if result:
        print(result)
        sys.exit(0)
    else:
        eprint("Could not find doctarget with name: \"" + docTarget + "\"")
        sys.exit(1)

def getTacticsRoot():
    procResult = subprocess.run(["rospack", "find", "roboteam_tactics"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    if not procResult.returncode == 0:
        print("Could not find roboteam_tactics!\n")
        sys.exit(1)
    else:
        # To remove the newline we use [:-1]
        tacticsPath = procResult.stdout.decode('utf8')[:-1]


    # skillsHeaderPath = os.path.join(tacticsPath, "include", "roboteam_tactics", "skills")

    return tacticsPath

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--of", help="Indicates the skill, condition, or tactic to look up")
    parser.add_argument("--roboteam-tactics-root", help="Can be used to force get_docs.py to use a specific directory to look for headers.")
    args = parser.parse_args()

    docTarget = None
    roboteam_tactics_root = None

    if args.of:
        docTarget = args.of

    if args.roboteam_tactics_root:
        roboteam_tactics_root = args.roboteam_tactics_root

    if not roboteam_tactics_root:
        roboteam_tactics_root = getTacticsRoot()

    # If no doc target is specified there is nothing to do here
    if not docTarget:
        sys.exit(0)

    getAndPrintDocumentation(docTarget, roboteam_tactics_root)
    # If nothing goes wrong, we terminate after this argument
    sys.exit(0)


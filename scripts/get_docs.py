#!/usr/bin/env python3

import subprocess
import sys
import os
import re

def removeFirstSpaceIfPresent(x):
    if len(x) > 0:
        if x[0] == " ":
            return x[1:]

    return x

def getAndPrintDocumentation(args, skillsHeaderPath):
    helpStr = """How to use:
- get_docs.py of [skillName]"""

    if len(args) < 2:
        print(helpStr)
        sys.exit(1)

    if not args[0] == "of":
        print(helpstr)
        sys.exit(1)

    desiredNode = args[1]
    
    found = False
    foundAt = ""
    for entry in os.listdir(skillsHeaderPath):
        if not os.path.isdir(entry) and entry.endswith(".h"):
            entryWithoutExtension = entry[:-2]

            if entryWithoutExtension == desiredNode:
                foundAt = os.path.join(skillsHeaderPath, entry)
                found = True
                break

    if not found:
        print("Could not find skill with name " + desiredNode)
        sys.exit(1)

    with open(foundAt, 'r') as f:
        contents = f.read();

    m = re.search("/\*.*?\*/.*/\*(.*)\*/", contents, re.DOTALL)

    if m:
        rawDocs = m.group(1)
        truncated = rawDocs.split("\n")

        # Removes anything up until the first star in every line
        truncated = [x.split("*", 1)[-1] for x in truncated]
        # Removes first space if present
        truncated = [removeFirstSpaceIfPresent(x) for x in truncated]
        # Removes empty lines
        truncated = [x for x in truncated if len(x) > 0]

        print("\n".join(truncated))

def getSkillsHeaderPath():
    procResult = subprocess.run(["rospack", "find", "roboteam_tactics"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    if not procResult.returncode == 0:
        print("Could not find roboteam_tactics!\n")
        sys.exit(1)
    else:
        # To remove the newline we use [:-1]
        tacticsPath = procResult.stdout.decode('utf8')[:-1]


    skillsHeaderPath = os.path.join(tacticsPath, "include", "roboteam_tactics", "skills")

    return skillsHeaderPath

if __name__ == "__main__":
    skillsHeaderPath = getSkillsHeaderPath()

    args = sys.argv[1:]

    if len(args) < 1:
        print("""How to use:
- get_docs.py of [skillname]""")
        sys.exit(1)

    if args[0] == "of":
        getAndPrintDocumentation(args, skillsHeaderPath)
        # If nothing goes wrong, we terminate after this argument
        sys.exit(0)
    else:
        print("unknown argument " + args[0])
        sys.exit(1)


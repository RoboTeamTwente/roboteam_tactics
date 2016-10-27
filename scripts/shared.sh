function assert_tactics_root {
    if [[ "$PWD" != *roboteam_tactics ]]; then
        printf "Invalid working directory; should be roboteam tactics root. Aborting.\n"
        exit 1
    fi
}

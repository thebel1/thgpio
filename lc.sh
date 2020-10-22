ls -1 | grep -E '\.(c|h)$' | xargs wc -l | { echo $'lines file\n--- ---'; cat; } | column -t

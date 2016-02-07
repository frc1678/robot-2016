{ echo "date +%T -s \\"; date | cut -d " " -f 4; } | sshpass -p "" ssh admin@roborio-1678-frc.local -t

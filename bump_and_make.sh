# bump version
awk -F'[."]' -e '/componentVersion/{ printf "%s\"%s\"%s%04d\"%s\n", $1, $2, $3"\""$4"."$5"."$6, int($7) + 1, $8; next; } { print; }' gpio.sc >./gpio.sc.new
mv gpio.sc gpio.sc.bak
mv gpio.sc.new gpio.sc

#make
make

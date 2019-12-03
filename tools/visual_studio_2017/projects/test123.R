library(ggplot2)
library(zoo)

test123 <- read.csv("~/git/ma_code/external/franka_proxy/tools/visual_studio_2017/projects/roll_mean.csv")

ggplot(test123, aes(x=1:length(test123$test))) +
  geom_line(aes(y=rollmean(filtered, 1, na.pad=TRUE)), color = "darkblue") +
  geom_line(aes(y=rollmean(test, 1, na.pad=TRUE)), color = "darkgreen")


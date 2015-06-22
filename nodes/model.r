
#summary(mydata)
#str(mydata)
#par(mfrow=c(1,4))
rt = mydata$rt
P = mydata$P
plot(P ~ rt, main="Relation in\nhard words", xlab="Num. characters", ylab="Performance")

par(mfrow=c(1,2))
lm.out1 = lm(L ~ d)
lm.out1
lm(formula = L ~ d)
plot(lm.out1$fitted, lm.out1$resid)
lm.out2 = lm(log(L) ~ log(d))
lm.out2
lm(formula = log(L) ~ log(d))
plot(lm.out2$fitted, lm.out2$resid)



# Model assuming easy question parameters, P(Correct) and response time (rt) in 'mydata'
c = 0.25    # Guessing parameter 
u = 0.8     # Upper bound
L1 = 45.8   # Number of characters in question
L2 = 23.4   # Number of characters in all responses
mydata <- read.csv("Hard.csv")

f <- function(a,b,rt,c,u,L1,L2) {c + ((u - c) / (1 + exp(-a * (-rt + b * (L1 + L2)))))}
st <- coef(nls(log(P) ~ log(f(a,b,rt,c,u,L1,L2)), mydata, start = list(a = -.1, b = -.1)))
fm <- nls(P ~ f(a,b,rt,c,u,L1,L2), data = mydata, start = st)
plot(P ~ rt, mydata)
loess_fit <- loess(P ~ rt, mydata)
lines(mydata$rt, predict(loess_fit), col = "blue")
lines(mydata$rt, predict(fm), col = "red")

# My model
u = 0.8     # Upper bound
L = 4.5     # Mean number of characters in word
mydata <- read.csv("Hard.csv")
plot(P ~ rt, mydata)
f <- function(a,b,rt,u,L) {u / (1 + exp(-a * (-rt + b * L)))}
st <- coef(nls(log(P) ~ log(f(a,b,rt,u,L)), mydata, start = list(a = -.1, b = -.1)))
# fit a non-linear regression
fm <- nls(P ~ f(a,b,rt,u,L), data = mydata, start = st)
lines(mydata$rt, predict(fm), col = "red")
# fit a loess line
loess_fit <- loess(P ~ rt, mydata)	
lines(mydata$rt, predict(loess_fit), col = "blue")



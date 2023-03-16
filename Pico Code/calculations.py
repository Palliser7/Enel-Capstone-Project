import math
saCopper =  1.7813919765234377e-05
saAluminum = 1.7813919765234377e-05
saBrass = 0.0011400908649750001
saSSteel = 0.0045603634599000005

mCopper = 0.0000624273
mAluminum = 0.00001908874
mBrass = 0.03065963
mSSteel = 0.2325354

AF = 0.0000000000

NF = 0.0000000000

FFcopper = 0.6 * mCopper*9.81
FFaluminum = 0.6 * mAluminum*9.81
FFBrass = 0.6 * mBrass*9.81
FFSSteel = 0.6 * mSSteel*9.81

ARCopper = 0.5 * 0.5 * 1.293 * 0.001**3 * 4.45348e-6 
ARAluminum = 0.5 * 0.5 * 1.293 * 0.001**3 * 4.45348e-6
ARBrass = 0.5 * 0.5 * 1.293 * 0.001**3 * 2.85023e-4
ARSSteel = 0.5 * 0.5 * 1.293 * 0.001**3 * 1.14009e-3

NFcopper = 0.001 * mCopper
NFaluminum = 0.001 * mAluminum
NFBrass = 0.001 * mBrass
NFSSteel = 0.001 * mSSteel

AFcopper = NFcopper + FFcopper + ARCopper + (mCopper*9.81)
AFaluminum = NFaluminum + FFaluminum + ARAluminum + (mAluminum*9.81)
AFBrass = NFBrass + FFBrass + ARBrass + (mBrass*9.81)
AFSSteel = NFSSteel + FFSSteel + ARSSteel + (mSSteel*9.81)

print(AFcopper)
print(AFaluminum)
print(AFBrass)
print(AFSSteel)

Bt = 0.9
V1 = 0.000653972
V2 = 0.001728
V3 = 0.003375
Uo = 4 * math.pi * 10**-7

x = Bt * V1/Uo
y = Bt * V2/Uo
z = Bt * V3/Uo

a = x*x + y*y + z*z
b = 1/a
c1 = b*x
c2 = b*y
c3 = b*z

print(c1)
print(c2)
print(c3)
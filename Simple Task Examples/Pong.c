

task main()
{

int LeftSqure = 0;
int RightSqure = 5;
int Xdir=1;
int TopSqure = 5;
int ButtomSqure = 0;
int Ydir = 1;


while(1==1)
{
	drawRect(LeftSqure, TopSqure, RightSqure, ButtomSqure);
	sleep(15);
	eraseDisplay();

	LeftSqure = LeftSqure + Xdir;
	RightSqure = RightSqure + Xdir;

	ButtomSqure = ButtomSqure + Ydir;
	TopSqure = TopSqure + Ydir;

	if((LeftSqure==0) || (RightSqure==177)){
		Xdir = -Xdir;
	}

	if((ButtomSqure==0) || (TopSqure==127)){
		Ydir = -Ydir;
  }
}
}

#include "Main.h"

void ControlLoop ( void )
{
    forward = GetJoystickAnalog( 1 , 2 ) ;  //ucitavanje komandi s joystick-a
    right = GetJoystickAnalog( 1 , 1 ) ;    //ucitavanje komandi s joystick-a
    yaw = GetJoystickAnalog( 1 , 4 ) ;      //ucitavanje komandi s joystick-a

    //implementacija mrtve zone na joysticu
        //uvijek postoji poneki mali signal oko nule
    if ( forward>10 ){
        forward=forward-10 ;
    } else if ( forward>-10 ){
        forward=0 ;
    } else {
        forward=forward+10 ;
    }
    if ( right>10 ) {
        right=right-10 ;
    } else if ( right>-10 ){
        right=0 ;
    } else {
        right=right+10 ;
    }
    if ( yaw>10 ) {
        yaw=yaw-10 ;
    } else if ( yaw>-10 ){
        yaw=0 ;
    } else {
        yaw=yaw+10 ;
    }

    //citanje brzina s enkodera
    e1 = GetQuadEncoder ( 1 , 2 );  //brzina prvog kotaca
    e2 = GetQuadEncoder ( 3 , 4 );  //brzina drugog kotaca
    e3 = GetQuadEncoder ( 5 , 6 );  //brzina treeg kotaca
    e4 = GetQuadEncoder ( 7 , 8 );  //brzina etvrtog kotaca

    //raunanje promjene brzine kotaa
    v1=e1-e1_old ;
    v2=e2-e2_old ;
    v3=e3-e3_old ;
    v4=e4-e4_old ;
    e1_old=e1 ;
    e2_old=e2 ;
    e3_old=e3 ;
    e4_old=e4 ;

    readUART();

    //racunanje referentne brzine za svaki kotac
    s1=(forward-right-yaw)/4 + r1_uart;
    s2=(forward+right+yaw)/4 + r2_uart;
    s3=(forward+right-yaw)/4 + r3_uart;
    s4=(forward-right+yaw)/4 + r4_uart;


    //racunanje regulacijskog odstupanja za svaki kotac
    reg1=v1-s1;
    reg2=v2-s2;
    reg3=v3-s3;
    reg4=v4-s4;


    //racunanje sume regulacijskih odstupanja
    e1_integral+=reg1;
    e2_integral+=reg2;
    e3_integral+=reg3;
    e4_integral+=reg4;

    if (s1!=0 && v1==0 && s2!=0 && v2==0 && s3!=0 && v3==0 && s4!=0 && v4==0 && zastavica == true){
      e1_integral = 0;
      e2_integral = 0;
      e3_integral = 0;
      e4_integral = 0;
    }

    //zastavica je bool globalna varijabla koja je na početku postavljena u false, ona sluzi kao upozorenje da robot stoji, a salju mu se naredbe da se krece
    //pri prvom pokretanju zastavica se postavlja u true, no cim se robot krene kretati vraca se u false
    //u slucaju da covjek stoji ispred senzora, a salju se naredbe da se robot pokrene
    //integral ce se resetirati u drugom loopu te se pokrenuti poslije kad se covjek odmakne 

    if (s1==0 && v1==0){
        e1_integral = 0;
    }
    if (s2==0 && v2==0){
        e2_integral = 0;
    }
    if (s3==0 && v3==0){
        e3_integral = 0;
    }
    if (s4==0 && v4==0){
        e4_integral = 0;
    }

    if (s1!=0 && v1==0 && s2!=0 && v2==0 && s3!=0 && v3==0 && s4!=0 && v4==0){
      zastavica = true;
    }
    else {
      zastvica = false;
    }

    //PI controller + pojaavanje reference
    fm1 =-s1*FFGAIN+(v1-s1)*PGAIN+e1_integral*IGAIN;
    fm2 =-s2*FFGAIN+(v2-s2)*PGAIN+e2_integral*IGAIN;
    fm3 =-s3*FFGAIN+(v3-s3)*PGAIN+e3_integral*IGAIN;
    fm4 =-s4*FFGAIN+(v4-s4)*PGAIN+e4_integral*IGAIN;


    //realni brojevi u integer, zaokruzivanje
    m1=(int)(fm1+0.5);
    m2=-(int)(fm2+0.5);
    m3=(int)(fm3+0.5);
    m4=-(int)(fm4+0.5);


    //svaki motor ima neku zonu neosjetljivosti na male brzine
        //linearizacija oko nule
    if ( m1>0 ){
        m1=m1+15 ;
    } else if (m1<0) {
        m1=m1-6 ;
    }
    if ( m2>0 ){
        m2=m2+15 ;
    } else if (m2<0) {
        m2=m2-6 ;
    }
    if ( m3>0 ){
        m3=m3+15 ;
    } else if (m3<0) {
        m3=m3-6 ;
    }
    if ( m4>0 ){
        m4=m4+15 ;
    } else if (m4>0) {
        m4=m4-6 ;
    }


    // ogranienje brzine motora izmeu -127 i 127 (to je oko 4 m/s)
    //saturacija
    if ( m1 > 127 ){
        m1 = 127 ;
    }
    if ( m1 < -127 ){
        m1 = -127 ;
    }
    if ( m2>127 ){
        m2=127 ;
    }
    if ( m2<-127 ){
        m2=-127 ;
    }
    if ( m3>127 ){
        m3=127 ;
    }
    if ( m3<-127 ){
        m3=-127 ;
    }
    if ( m4>127 ){
        m4=127 ;
    }
    if ( m4<-127 ){
        m4=-127 ;
    }

    //slanje upravljake brzine na motore
    SetMotor ( 2 , m1 ) ;
    SetMotor ( 3 , m2 ) ;
    SetMotor ( 4 , m3 ) ;
    SetMotor ( 5 , m4 ) ;


    WriteSerialPort(1, 0xAA);
    WriteSerialPort(1, 0xFF);
    unsigned char checksum=0;
    WriteSerialPort(1, 0x30);
    checksum+=0x30;
    WriteSerialPort(1, 0x08);
    checksum+=0x08;
    unsigned char* p1=(unsigned char*)&v1;
    WriteSerialPort(1,p1[0]);
    checksum+=p1[0];
    WriteSerialPort(1,p1[1]);
    checksum+=p1[1];
    unsigned char* p2=(unsigned char*)&v2;
    WriteSerialPort(1,p2[0]);
    checksum+=p2[0];
    WriteSerialPort(1,p2[1]);
    checksum+=p2[1];
    unsigned char* p3=(unsigned char*)&v3;
    WriteSerialPort(1,p3[0]);
    checksum+=p3[0];
    WriteSerialPort(1,p3[1]);
    checksum+=p3[1];
    unsigned char* p4=(unsigned char*)&v4;
    WriteSerialPort(1,p4[0]);
    checksum+=p4[0];
    WriteSerialPort(1,p4[1]);
    checksum+=p4[1];
    checksum=0-checksum;
    WriteSerialPort(1,checksum);

}

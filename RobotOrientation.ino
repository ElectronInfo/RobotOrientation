#include <Adafruit_BNO055.h>
#include <VL53L0X.h>

const uint8_t bumperPinL = 2;	// Пин левого датчика в бампере, появляется высокий уровень (HIGH) при срабатывании
const uint8_t bumperPinR = 3;	// Пин правого датчика в бампере, появляется высокий уровень (HIGH) при срабатывании

const uint8_t motorPinL1 = 4;	// Пин левого мотора
const uint8_t motorPinL2 = 5;	// Пин левого мотора
const uint8_t motorPinR1 = 6;	// Пин правого мотора
const uint8_t motorPinR2 = 7;	// Пин правого мотора

const uint8_t vlXPinF = 12;		// Пин XSHUT датчика расстояния спереди
const uint8_t vlXPinR = 11;		// Пин XSHUT датчика расстояния справа


const uint8_t vlAddressF = 0x30;	// Временный адрес датчика расстояния спереди
const uint8_t vlAddressR = 0x31;	// Временный адрес датчика расстояния справа

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);	// Адрес датчика ориентации (0x29 или 0x28)

const int distBarrier = 120;		// Минимальное расстояние до препятствия спереди в мм


// Раскомментируйте эту строку, чтобы использовать режим дальнего действия.
// Это повышает чувствительность датчика и расширяет его
// потенциальный диапазон, но увеличивает вероятность получения
// неточных показаний из-за отражений от объектов,
// отличных от намеченной цели. Он лучше всего работает в темных условиях.

//#define LONG_RANGE


// Раскомментируйте ОДНУ из этих двух строк, чтобы получить
// - более высокую скорость за счет более низкой точности ИЛИ
// - более высокую точность за счет более низкой скорости

//#define HIGH_SPEED
//#define HIGH_ACCURACY





VL53L0X sensorF;
VL53L0X sensorR;


int8_t direction = 0;	// Текущее направление, если смотреть на робота сверху от начального положения: 0 - вверх, 1 - вправо, 2 - вниз, 3 - влево
uint32_t time;

int32_t x = 0;	// Приблизительные координаты, положительное число вправо, отрицательное влево
int32_t y = 0;	// Приблизительные координаты, положительное число вниз, отрицательное вверх

int32_t timeTurn = 999;
int32_t distTurn = 9999;
uint32_t timeForward = 0;


bool lastTurn = true;
int degree = 0;
int8_t barrierType = 0;




void leftB()
{
	digitalWrite(motorPinL2, LOW);
	pinMode(motorPinL2, INPUT);

	pinMode(motorPinL1, OUTPUT);
	digitalWrite(motorPinL1, HIGH);
}

void leftF()
{
	digitalWrite(motorPinL1, LOW);
	pinMode(motorPinL1, INPUT);

	pinMode(motorPinL2, OUTPUT);
	digitalWrite(motorPinL2, HIGH);
}

void leftStop()
{
	digitalWrite(motorPinL1, LOW);
	pinMode(motorPinL1, INPUT);

	digitalWrite(motorPinL2, LOW);
	pinMode(motorPinL2, INPUT);
}

void rightB()
{
	digitalWrite(motorPinR2, LOW);
	pinMode(motorPinR2, INPUT);

	pinMode(motorPinR1, OUTPUT);
	digitalWrite(motorPinR1, HIGH);
}

void rightF()
{
	digitalWrite(motorPinR1, LOW);
	pinMode(motorPinR1, INPUT);

	pinMode(motorPinR2, OUTPUT);
	digitalWrite(motorPinR2, HIGH);
}

void rightStop()
{
	digitalWrite(motorPinR1, LOW);
	pinMode(motorPinR1, INPUT);

	digitalWrite(motorPinR2, LOW);
	pinMode(motorPinR2, INPUT);
}

void updateXY()
{
	uint32_t diffTime = millis() - time;
	
	uint32_t mmDist = (diffTime * 260)/1000 + 25;
	
	switch(direction)
	{
	case 0:
		y -= mmDist;
		break;
		
	case 1:
		x += mmDist;
		break;
		
	case 2:
		y += mmDist;
		break;
		
	case 3:
		x -= mmDist;
		break;
	}
}

void forward()
{
	leftF();
	rightF();

	time = millis();
}

void back()
{
	leftB();
	rightB();
}

void left()
{  
	leftB();
	rightF();

	updateXY();
}

void right()
{  
	rightB();
	leftF();

	updateXY();
}

void stop()
{  
	leftStop();
	rightStop();

	updateXY();
}


float getAngle()
{
	sensors_event_t orientationData ;
	bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
	return orientationData.orientation.x;
}

int dist()
{
	if (!sensorF.timeoutOccurred())
	{
		return sensorF.readRangeSingleMillimeters();
	}
	else
	{
		return 9999;
	}
}

int distR()
{
	if (!sensorR.timeoutOccurred())
	{
		return sensorR.readRangeSingleMillimeters();
	}
	else
	{
		return 9999;
	}
}


void mydelay(unsigned long millisecond)
{
	unsigned long pMillis = millis();
	
	while(millis() - pMillis < millisecond)
	{
		//
	}
}

void mydelayCheck(unsigned long millisecond)	// Вместо своей процедуры можно использовать стандартную delay() + yield()
{
	unsigned long pMillis = millis();
	
	while(millis() - pMillis < millisecond)
	{
		distTurn = max(dist(), dist());
		
		if(digitalRead(bumperPinL) || digitalRead(bumperPinR) || distTurn < distBarrier)
		{
			timeTurn = millis() - pMillis;
			
			break;
		}
		
		//
	}
}

int diffAngle(int x, int y)
{
	int diff = x - y;
	
	diff += (diff>180) ? -360 : (diff<-180) ? 360 : 0;
	return diff;
}

void delayDist(int cm)
{
	delay((cm - 1)*38-12);
}

int normalizeAngle(int deg)
{
	if(deg > 360) 
		deg -= 360;
	else if(deg < 0) 
		deg += 360;
	
	return deg;
}

void turn(bool rightTurn, int angle = 90)
{
	if(rightTurn)
	{
		right();
	}
	else
	{
		left();
	}

	degree += rightTurn ? angle : -angle;
	
	direction += rightTurn ? angle/90 : -(angle/90);
	if(direction < 0) direction = 4 + direction;
	direction = direction%4;
	
	degree = normalizeAngle(degree);
	
	int tDegree = degree - 180;

	
	while(abs(diffAngle(tDegree, (int)getAngle()-180)) > 15)
	{
		//
	}
}

void turnLeft()
{
	turn(false);
}

void turnRight()
{
	turn(true);
}

void checkBarrierWhile(int mm = distBarrier)	// Ждать, пока расстояние не будет меньше указанного или не произойдет столкновение
{
	while(!digitalRead(bumperPinL) && !digitalRead(bumperPinR) && max(dist(), dist()) >= mm);
}

int8_t checkBarrier(int mm = distBarrier)	// Проверка столкновений и минимального расстояния, возвращает причину срабатывания: 1 - левый датчик, 2 - правый датчик, 4 - датчик расстояния, и их комбинации в зависимости от суммы, например: 3 - левый и правый датчик (хотя это может никогда не случиться, так как проверка единовременная без задержек и повторной проверки через некоторое время)
{
	barrierType = 0;
	if(digitalRead(bumperPinL)) barrierType += 1;
	if(digitalRead(bumperPinR)) barrierType += 2;
	if(max(dist(), dist()) < mm)	barrierType += 4;
	
	return barrierType;
}

void align()	// Выравнивание с помощью притормаживания левого или правого колеса, если угол не соответствует необходимому (увеличивает износ моторов из-за постоянного пуска и остановки, нужно переделывать в изменение скорости вращения левого/правого колеса, а не включено/выключено)
{
	int tDegree = degree - 180;

	int diffAng = diffAngle(tDegree, (int)getAngle()-180);
	
	if(diffAng > 0)
	{
		rightStop();
		mydelay(diffAng > 1 ? 50 : 25);
		rightF();
	}
	else if(diffAng < 0)
	{
		leftStop();
		mydelay(diffAng < -1 ? 50 : 25);
		leftF();
	}
}

void forwardFront(int cm)	// Движение вперед, пока расстояние не будет меньше указанного или не произойдет столкновения
{
	forward();
	
	cm = cm * 10;
	
	while(max(dist(), dist()) > max(distBarrier, cm) && !digitalRead(bumperPinL) && !digitalRead(bumperPinR))
	{
		align();
	}
}

unsigned long cmToMillis(int cm)	// Перевод расстояния(см) во время(мс), необходимое для преодоления этого расстояния
{
	return (cm - 1)*38-12;
}

void forwardDist(int cm)	// Движение вперед на определенное расстояние(см) или не произойдет столкновение
{
	forward();
	
	unsigned long distMillis = cmToMillis(cm);
	
	unsigned long pMillis = millis();
	
	while(!checkBarrier() && millis() - pMillis < distMillis)
	{
		align();
	}
}

void snake(bool wallright, int cm = 0)	// Движение змейкой, wallright - true, если стена без препятствий преимущественно перед поворотом направо (то есть робот двигаясь к стене повернет направо, а затем ещё раз направо, а от двигаясь от ровной стены повернет налево), false - перед поворотом налево; дополнительный параметр расстояния, если нужно от этой стены двигаться змейкой не более указанного расстояния (то есть разворачиваться заранее, даже если нет препятствий), при движении к стене движение происходит независимо от указанного расстояния
{
	lastTurn = !lastTurn;
	
	timeForward = millis();
	
	if(cm == 0 || (!wallright && !lastTurn) || (wallright && lastTurn))
	{
		forward();
		
		int countBar = 0;
		
		while(1)
		{
			while(!checkBarrier())
			{
				align();
			}

			if((wallright && barrierType == 1 && lastTurn) || (!wallright && barrierType == 1 && ((lastTurn && countBar == 0) || !lastTurn)))	// Если левый датчик и нужно направо (то есть к стене), то объезжаем || Если левый датчик и (налево или направо первый раз), то объезжаем
			{
				back();
				mydelay(500);
				turn(true);
				forward();
				mydelay(500);
				turn(false);
				forward();
				
				countBar++;
			}
			else if((wallright && barrierType == 2 && ((!lastTurn && countBar == 0) || lastTurn)) || (!wallright && barrierType == 2 && !lastTurn))	//	|| Если правый датчик и нужно налево (то есть к стене), то объезжаем
			{
				back();
				mydelay(500);
				turn(false);
				forward();
				mydelay(500);
				turn(true);
				forward();
				
				countBar++;
			}
			else
			{
				if(barrierType <= 2)
				{
					back();
					mydelay(500);
				}
				
				break;
			}
		}
	}
	else
	{
		forwardDist(cm);
	}
	
	timeForward = millis() - timeForward;

	turn(lastTurn);

	forward();

	mydelayCheck(500);

	turn(lastTurn);
	
	forward();
}

void stopError()	// Остановиться и не продолжать дальше
{
	stop();
	while(1);
}

void forwardWhileRight(int cmRight, int cmFront = 15)	// Двигаться вперед, пока расстояние справа не станет больше указанного или расстояние спереди не станет меньше указанного или не произойдет столкновение
{
	forward();
	
	while(!digitalRead(bumperPinL) && !digitalRead(bumperPinR) && dist() > cmFront*10 && distR() < cmRight*10)
	{
		align();
	}
}



void setup()
{  
	digitalWrite(vlXPinF, LOW);
	digitalWrite(vlXPinR, LOW);
	pinMode(vlXPinF, OUTPUT);
	pinMode(vlXPinR, OUTPUT);
	


	Serial.begin(115200);



	if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
	{
		/* Проблема обнаружения BNO055... Проверьте соединения */
		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		while (1);
	}

	delay(1000);

	/* Использовать внешний кварц для лучшей точности */
	bno.setExtCrystalUse(true);


	pinMode(vlXPinF, INPUT);
	delay(500);

	sensorF.setTimeout(500);
	if (!sensorF.init())
	{
		Serial.println("Failed to detect and initialize sensorF!");
		while (1) {}
	}

	sensorF.setAddress(vlAddressF);

	delay(100);
	pinMode(vlXPinR, INPUT);
	delay(500); 

	sensorR.setTimeout(500);
	if (!sensorR.init())
	{
		Serial.println("Failed to detect and initialize sensorR!");
		while (1) {}
	}

	sensorR.setAddress(vlAddressR);


#if defined LONG_RANGE
	// lower the return signal rate limit (default is 0.25 MCPS)
	sensorF.setSignalRateLimit(0.1);
	// increase laser pulse periods (defaults are 14 and 10 PCLKs)
	sensorF.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
	sensorF.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
	// reduce timing budget to 20 ms (default is about 33 ms)
	sensorF.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
	// increase timing budget to 200 ms
	sensorF.setMeasurementTimingBudget(200000);
#endif


	sensorF.setSignalRateLimit(0.5);
	
	
	// Автоматический расчет прямого угла относительно стены спереди
	sensorF.setMeasurementTimingBudget(500000);
	sensorR.setMeasurementTimingBudget(500000);

	left();
	mydelay(150);
	stop();
	mydelay(500);

	float distL = dist()+140;
	float angleL = getAngle();
	right();
	mydelay(300);
	stop();
	mydelay(500);
	float distR = dist()+140;
	float angleR = getAngle();


	double angleLR = angleL - angleR;

	angleLR += (angleLR>180) ? -360 : (angleLR<-180) ? 360 : 0;

	angleLR = abs(angleLR);

	double lenC = sqrt(sq(distL)+sq(distR)-2*distL*distR*cos(radians(angleLR)));

	double per = (distL + distR + lenC) / 2;
	double lenH = 2/lenC * sqrt(per*(per-distL)*(per-distR)*(per-lenC));

	double diffAngles = degrees(acos(lenH/distR));

	degree = normalizeAngle(round(angleR - diffAngles));
	
	sensorF.setMeasurementTimingBudget(33000);
	sensorR.setMeasurementTimingBudget(33000);


	turn(true, 180);	// Повернуться на 180 градусов по часовой стрелке (встать задней частью робота к стене)

	forwardFront(20);	// Движение вперед, пока расстояние до препятствия спереди не будет 20 см или меньше
	turnLeft();			// Повернуть налево на 90 градусов (после поворота, код продолжает исполняться дальше, но моторы остаются в том же состоянии, поэтому после поворота нужно вызывать процедуру движения вперед, назад или остановки)
	
	forwardWhileRight(100);	// Двигаться вперед, пока расстояние справа не станет больше 100 см
	mydelayCheck(600);		// Проехать ещё 600 миллисекунд, чтобы задняя половина пылесоса также проехала препятствие
	turnRight();			// Повернуть направо на 90 градусов
	forward();				// Двигаться вперед
	mydelayCheck(2000);		// Проехать ещё 2 секунды, чтобы свободное расстояние справа точно закончилось
	forwardWhileRight(20);	// Двигаться вперед, пока расстояние справа не станет больше 20 см
	
	turnLeft();			// Повернуть налево
	forward();			// Двигаться вперед	
	
	lastTurn = false;	// Был налево, потом будет направо (для движения змейкой, чтобы первый поворот был направо)
	while(1)
	{
		snake(true, 160);	// Движение змейкой (стена перед поворотом направо, от неё отъезжать максимум 160 см)
		
		if(!lastTurn && timeTurn < 200)	// Как только поворот налево стал происходить меньше 200 миллисекунд, движение змейкой завершить (уперлись в стену)
		break;
	}
	
	forwardFront(16);	// Движение вперед, пока расстояние до препятствия спереди не будет 16 см или меньше
	turnLeft();
	forward();
	lastTurn = true;
	
	while(1)
	{
		snake(true, 250);	// Стена без препятствий преимущественно перед поворотом направо, максимум * см вправо
		
		if(!lastTurn && distTurn < 200)	// Как только при повороте налево, расстояние спереди (справа после полного поворота) стало меньше 20 см, движение змейкой завершить (уперлись в стену)
		break;
	}
	
	forwardWhileRight(20);	// Двигаться вперед, пока расстояние справа не станет больше 20 см
	back();					// Отъехать назад
	mydelayCheck(500);		// В течение 500 мс
	turnRight();
	forwardFront(14);
	turnRight();
	forwardFront(50);
	stop();		// Остановиться
	while(1);	// Не продолжать выполнение кода дальше (для примера)
	
	
	// Другие варианты движений
	
	// Змейка
	lastTurn = false;
	while(1)
	{
		snake(true, 175);	// Стена без препятствий преимущественно перед поворотом направо, максимум 175 см вправо
		
		if(lastTurn && dist() < 1000)	// Как только расстояние спереди после поворота направо стало меньше 100 см, перейти в другой режим змейки
		break;
	}
	
	// Змейка
	lastTurn = true;
	while(1)
	{
		snake(true);	// Стена без препятствий преимущественно перед поворотом направо
		
		if(!lastTurn && distTurn < 1300)	// Как только во время поворота налево, расстояние спереди (справа после полного поворота) стало меньше 130 см, движение змейкой завершить
		break;
	}
	
	// Змейка
	lastTurn = true;
	while(1)
	{
		snake(false, 80);	// Стена перед поворотом налево, максимум * см вправо
		
		if(!lastTurn && timeForward > 7000)	// Время движения в одну сторону составило больше 7 секунд
		break;
	}
	
	
	forwardDist(40);	// Проехать 40 см вперед
	turnRight();
	forwardFront(18);
	stop();
	
	
	x = 0;	// Обнулить переменные координат для дальнейшего использования
	y = 0;
	
	lastTurn = true;	// Был направо, потом будет налево (для движения змейкой, чтобы первый поворот был налево)
	while(1)
	{
		snake(true, y < 2000 ? 160 : 215);	// Если координата y от 0 до ~2000 мм (2 м, знак > -2000 или  < 2000 зависит от изначального направления робота), то двигаться змейкой максимум от стены 160 см, а после 2 м 215 см 
		
		if(y > 2300 && lastTurn && timeTurn < 200)	// Как только отъехали от момента сброса координат больше ~2.3м и время поворота стало занимать меньше 200 мс, то завершить движение змейкой (уперлись в препятствие)
		break;
	}
}


void loop()
{
	
}

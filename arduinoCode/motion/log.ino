/* Author: Maarten Weyn
   Repo: https://github.com/maartenweyn/DajiJug
   */
void logString(String message)
{
  if (logUSB)
    Serial.print(message);
    
  if (logBT)
    Serial1.print(message);
}

void logInt(int value)
{
  if (logUSB)
    Serial.print(value);
    
  if (logBT)
    Serial1.print(value);
}
  
  

double stringToNumber(String thisString) {
  int i, length;
  double value = 0.0;
  length = thisString.length();
  int factor = 1;
  int startPos = 0;
  boolean decimal = false;
  int decimalPosition = 0;
  if (thisString.charAt(0) == '-')
  {
    factor = -1;
    startPos = 1;
  } else if (thisString.charAt(0) == 'x')
  {
      return NAN;
  }
  for(i=startPos; i<length; i++) {
    if (thisString.charAt(i) == '.')
    {
      decimal = true;
    } else {
      int digit = thisString.charAt(i)-(int) '0';
      if (decimal) 
      {
          decimalPosition = decimalPosition + 1;
          value = value + (digit / (10^decimalPosition));
      } else {      
          value = (10*value) + digit;
      }
    }
  }
  
  value *= factor;
  return value;
}

void splitString (String input, int nrOfElements, double result[])
{
  int counter = 0;
  char* str;
  char inputArray[input.length()+1];
  char* inputPointer = inputArray;
  input.toCharArray(inputArray,input.length()+1);
  
  while ((str = strtok_r(inputPointer, ",", &inputPointer)) != NULL)
  {
//    Serial.print(counter);
//    Serial.print(": ");
//    Serial.print(inputPointer);
//    Serial.print(" -> ");
//    Serial.println(str);
    result[counter++] = stringToNumber(str);
  }
}

#if __cplusplus < 202002L
	#error "Well gosh-tootin' darn. You done don't got none c++20!"
#endif
// MacroScript

#include <iostream>
#include <string>
#include <cstring>
#include <cassert>
#include <stdio.h>
#include <map>
#include <queue>
#include <functional>


struct Command;
struct Macro;


typedef std::function<bool(Macro&)> extfun_t;

class not_implemented_error : public std::logic_error // THANKS, STACKOVERFLOW
{
public:
    not_implemented_error() : std::logic_error("Function not yet implemented") { };
};


enum Type {
	NONE,
	STRING,
	NUMBER,
	BOOLEAN,
	FUNCTION,
	EXTFUN
};


struct Object {
	Type type = NONE;
	
	std::string name;
	std::string string;
	double number;
	bool boolean;
	std::vector <Command> fun;
	size_t sP = 0; // stack pointer :D
	extfun_t extFun;

	Object(){
	
	}
	
	Object(const Object& o){
		type = o.type;
		string = o.string;
		number = o.number;
		boolean = o.boolean;
		extFun = o.extFun;
		fun = o.fun;
	}
	
	Object(std::string s){
		*this = s;
	}
	
	Object(double d){
		*this = d;
	}
	
	Object(bool b){
		*this = b;
	}
	
	Object(extfun_t f){
		*this = f;
	}
	
	~Object(){
	
	}
	
	double& getNum() {
		assert(type == NUMBER);
		return number;
	}
	
	std::string& getString() {
		assert(type == STRING);
		return string;
	}
	
	bool& getBool() {
		assert(type == BOOLEAN);
		return boolean;
	}
	
	void operator=(double x){
		type = NUMBER;
		number = x;
	}
	
	void operator=(std::string x){
		type = STRING;
		string = x;
	}
	
	void operator=(bool x){
		type = BOOLEAN;
		boolean = x;
	}
	
	void operator=(const char* x){
		string = x;
		type = STRING;
	}
	
	void operator=(extfun_t fun){
		type = EXTFUN;
		extFun = fun;
	}
	
    Object& operator=(const Object& o){
		type = o.type;
		string = o.string;
		number = o.number;
		boolean = o.boolean;
		extFun = o.extFun;
		fun = o.fun;
        return *this;
    }

	std::string toString(){
		if (type == BOOLEAN){
			return (std::string)"Bool " + (boolean ? "true" : "false") + " " + name;
		}
		else if (type == STRING){
			return (std::string)"String \"" + string + "\"" + " " + name;
		}
		else if (type == NUMBER){
			return (std::string)"Number " + std::to_string(number) + " " + name;
		}
		else if (type == EXTFUN){
			return (std::string)"External (C++) function" + " " + name;
		}
		else if (type == FUNCTION){
			return (std::string)"Internal function" + " " + name;
		}
		else if (type == NONE){
			return (std::string)"NULL";
		}
		else {
			return "Error: Invalid type.";
		}
	}
	
	bool call_internal(Macro& m);
	
	bool operator()(Macro& m){
		assert(type == FUNCTION || type == EXTFUN);
		if (type == FUNCTION){
			return call_internal(m);
		}
		else {
			// the cool thing about assertions is that we know that if the code reaches this point it's an EXTFUN, because it can't be anything but FUNCTION or EXTFUN.
			return extFun(m);
		}
	}
	
	void operator+=(Command c);
};

std::ostream& operator<<(std::ostream& os, Object &obj) { /* THANKS, STACKOVERFLOW */
    return os << obj.toString();
}


struct Command {
	std::string command;
	std::vector<Object> args;
	bool loaded = false;

	Command& operator+=(Object* thing){ // It is expected that the object has been heap allocated and must be destructed intelligently.
		args.push_back(*thing);
		return *this;
	}
	
	Command& operator+=(Object& thing){ // It is expected that this has *not* been heap allocated, and will thus have to be copied to be safely used here.
		Object* copy = new Object(thing);
		return *this += copy;
	}
	
	bool operator==(std::string thing){
		return command == thing;
	}
	
	void operator=(std::string thing){
		command = thing;
	}
	
	Object& operator[](size_t x){
		return args[x];
	}
};


struct CleverFile { // it's basically iostream, but less suck
	FILE* file;
	bool newLine = false;

	CleverFile(const char* fname){
		file = fopen(fname, "r");
	}

	std::string ReadUntil(char thang){
		std::string ret;
		char c = fgetc(file);
		bool escape = false;
		while (((c != thang && c != 10) || escape) && !feof(file)){ // This is line-based, so processing ends at a newline
			if (c == '\\' && !escape){
				escape = true;
			}
			else{
				ret += c;
				escape = false;
			}
			c = fgetc(file);
		}
		if (c == 10){
			newLine = true;
		}
		return ret;
	}
	
	bool LineEnded(){ // Whether or not a line ended
		bool wasLineEnded = newLine;
		newLine = false;
		return wasLineEnded || feof(file);
	}
	
	operator bool(){
		return !feof(file);
	}
	
	char peek(){
		char r = fgetc(file);
		ungetc(r, file);
		return r;
	}
	
	char read(){
		return fgetc(file);
	}
};


bool isNum(std::string thing){
	bool hasDot = false;
	for (char x : thing){
		if (x == '.'){
			if (hasDot){
				return false;
			}
			hasDot = true;
		}
		else if (x < '0' || x > '9'){
			return false;
		}
	}
	return thing.size() > 0;
}


class Macro {
public:
	CleverFile file;
	std::vector <Object> stack;
	std::map <std::string, Object> global;

	Command next(){
		Command ret;
		ret = file.ReadUntil(' '); // Set command
		while (!file.LineEnded()){
			Object o;
			if (file.peek() == '"'){
				file.read();
				std::string s = file.ReadUntil('"');
				o = s;
				if (!s.size()){
					continue;
				}
			}
			else{
				std::string thing = file.ReadUntil(' ');
				if (thing.size() == 0){
					continue;
				}
				if (isNum(thing)) {
					o = std::stod(thing);
				}
				else if (thing == "true"){
					o = true;
				}
				else if (thing == "false"){
					o = false;
				}
				else {
					o = thing;
				}
			}
			ret += o;
		}
		return ret;
	}
	
	std::queue <Command> commands;
	
	Macro(const char* fname) : file (fname) {
		while (file){
			commands.push(next());
		}
	}
	
	int funMode = 0;
	Object functionBuffer;
	
	bool Execute(Command& c){ // Return true if the command is complete
		if (funMode > 0){
			functionBuffer.type = FUNCTION;
			if (c == "fun"){
				funMode ++;
			}
			else if (c == "endFun"){
				funMode --;
			}
			else {
				functionBuffer += c; // FINE AT THIS POINT
			}
			if (funMode == 0){
				PushStack(functionBuffer); // FINE AT THIS POINT
				PopStack()(*this);
			}
			return false;
		}
		if (!c.loaded){
			for (Object& o : c.args){ // Re-push everything to stack
				PushStack(o);
			}
			c.loaded = true;
		}
		if (c == "call"){
			Object& _f = PeekStack();
			assert ((_f.type == FUNCTION) || (_f.type == EXTFUN) || (_f.type == STRING));
			Object& fun = _f;
			if (_f.type == STRING){
				std::string fname = _f.getString();
				assert(global.contains(fname));
				fun = global[fname];
			}
			ShiftStack();
			bool ret = fun(*this);
			UnshiftStack();
			if (ret){
				PopStack();
			}
			return !ret;
		}
		else if (c == "getStored"){
			Object& name = PopStack();
			assert(name.type == STRING);
			std::string id = name.getString();
			assert(global.contains(id));
			Object f = global[id];
			PushStack(f);
		}
		else if (c == "store"){
			Object& name = PopStack();
			assert(name.type == STRING);
			std::string id = name.getString();
			global[id] = PopStack();
		}
		else if (c == "fun"){
			funMode ++;
		}
		else if (c == "pStack"){
			pStack();
		}
		else if (c == "pop"){
			PopStack();
		}
		return false;
	}

	bool Execute() { // Returns false if it's finished, else true
		if (commands.size() == 0){
			return false;
		}
		if (!Execute(commands.front())){
			commands.front().loaded = false; // Unload the command
			commands.pop();
		}
		return true;
	}
	
	void pStack(){
		std::cout << "==== Macro++ (Tyler++ v3) Stack ====" << std::endl;
		size_t stShift = 0;
		while (StackSize()){
			ShiftStack();
			stShift ++;
			std::cout << PeekStack() << std::endl;
		}
		shift -= stShift;
		std::cout << "============= Stack end ============" << std::endl;
	}
	
	Object& PopStack(){
		Object& ret = PeekStack();
		stack.erase(stack.end() - shift);
		return ret;
	}
	
	Object& PeekStack() {
		assert((long)stack.size() - 1 - shift >= 0);
		return stack[stack.size() - 1 - shift];
	}
	
	size_t StackSize() {
		return stack.size() - shift;
	}
	
	void PushStack(Object& thing){
		stack.insert(stack.end() - shift, thing);
	}
	
	long shift = 0;
	
	void ShiftStack() { // Mask the current item in the stack by increasing the "window" by one
		shift ++;
	}
	
	void UnshiftStack() { // Unmask the last masked item in the stack by decreasing the "window" by one
		assert(shift > 0);
		shift --;
	}
};


bool Object::call_internal(Macro& m){
	std::cout << "COMMAND " << sP << std::endl;
	if (sP < fun.size()){
		if (!m.Execute(fun[sP])){
			std::cout << "COMMAND CUMPLET " << fun[sP].command << std::endl;
			fun[sP].loaded = false;
			sP ++;
		}
	}
	else {
		sP = 0;
		return true;
	}
	return false;
}

void Object::operator+=(Command c){
	fun.push_back(c);
	type = FUNCTION;
}
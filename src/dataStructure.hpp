/**
 * dataStructure class holds the data for the controller process and provides
 * convenience functions to help expose the data
 */

class dataStructure{
	private:
		float _roll, _pitch, _yaw;
	public:
		// Data members
		
		// Member Functions
		/**
		 * dataStructure constructor.  Constructs an empty dataStructure with
		 * fields initialized to 0.
		 */
		dataStructure();
		/**
		 * Copy constructor.  Constructs a dataStructure with fields
		 * initialized to be equal in value to the fields of the
		 * referenced dataStructure.
		 */
		dataStructure(const dataStructure& reference);
		/**
		 * Roll getter member.  Returns the current roll of the system
		 * in degrees clockwise from up in the range -180 to 180.
		 */
		inline float getRoll(){
			return _roll;
		}
		/**
		 * Pitch getter member.  Returns the current pitch of the system
		 * in degrees up from the horizon in the range -180 to 180.
		 */
		inline float getPitch(){
			return _pitch;
		}
		/**
		 * Yaw getter member.  Returns the current yaw of the system in degrees
		 * right from forward in the range -180 to 180.
		 */
		inline float getYaw(){
			return _yaw;
		}
		/**
		 * ZMQ message generation member.  Returns a pointer to the message
		 * buffer for ZMQ to process.
		 */
		void* getZMQMessage();
};

/*
 * Copyright (c) 2024 Federal University of Piauí
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Thiago Allisson <thiago.allisson@ufpi.edu.br>
 */

#include "app-tag.h"

#include "ns3/tag.h"
#include "ns3/uinteger.h"

namespace ns3
{
namespace lorawan
{

NS_OBJECT_ENSURE_REGISTERED(AppTag);

TypeId
AppTag::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::AppTag").SetParent<Tag>().SetGroupName("lorawan").AddConstructor<AppTag>();
    return tid;
}

TypeId
AppTag::GetInstanceTypeId() const
{
    return GetTypeId();
}

AppTag::AppTag()
    : m_msgType(NONE)
{
}

AppTag::AppTag(uint8_t msgType)
    : m_msgType(msgType)
{
}

AppTag::~AppTag()
{
}

uint32_t
AppTag::GetSerializedSize() const
{
    return 1;
}

void
AppTag::Serialize(TagBuffer i) const
{
    i.WriteU8(m_msgType);
}

void
AppTag::Deserialize(TagBuffer i)
{
    m_msgType = i.ReadU8();
}

void
AppTag::Print(std::ostream& os) const
{
    os << m_msgType;
}

uint8_t
AppTag::GetMsgType() const
{
    return m_msgType;
}

void
AppTag::SetMsgType(uint8_t msgType)
{
    m_msgType = msgType;
}

} // namespace lorawan
} // namespace ns3
